use r2r::geometry_msgs::msg::TransformStamped;
use r2r::tf2_msgs::msg::TFMessage;
use r2r::{Context, Node};
use r2r::std_msgs::msg::Bool;
use r2r::std_srvs::srv::Trigger;
use std::sync::{Arc, Mutex};
//use std::time::Duration;
use futures::stream::StreamExt;
use futures::future;
use cgmath::{Deg, Rad, Euler, Quaternion, Vector3};

#[derive(Clone, Default)]
struct State {
    // markers 0 and 1 define the facade position
    marker_0: Option<TransformStamped>,
    marker_1: Option<TransformStamped>,

    // markers 2 and 15 define the gantry position
    marker_2: Option<TransformStamped>,
    marker_15: Option<TransformStamped>,

    // marker 5 is the agv
    marker_5: Option<TransformStamped>,

    // computed results
    facade_transform: Option<TransformStamped>,
    gantry_transform: Option<TransformStamped>,
    agv_transform: Option<TransformStamped>,

    // locked results
    locked_facade_transform: Option<TransformStamped>,
    locked_gantry_transform: Option<TransformStamped>,
}

fn update_or_set(new: TransformStamped, maybe_old: &mut Option<TransformStamped>) {
    if let Some(x) = maybe_old.as_mut() {
        *x = filter_transform(new, x.clone());
    } else {
        println!("marker is live {}", new.child_frame_id);
        *maybe_old = Some(new)
    }
}

/// apply a low-pass filter to the position in the camera frame on incoming data
fn filter_transform(new: TransformStamped, old: TransformStamped) -> TransformStamped {
    let mut new_transform = new.clone();

    let smooth = 10.0;

    let nx = new.transform.translation.x;
    let ny = new.transform.translation.y;
    let nz = new.transform.translation.z;

    let ox = old.transform.translation.x;
    let oy = old.transform.translation.y;
    let oz = old.transform.translation.z;

    let diff_x = (nx - ox) / smooth;
    let diff_y = (ny - oy) / smooth;
    let diff_z = (nz - oz) / smooth;

    new_transform.transform.translation.x = ox + diff_x;
    new_transform.transform.translation.y = oy + diff_y;
    new_transform.transform.translation.z = oz + diff_z;

    new_transform
}

/// filter out bad measurements
#[allow(dead_code)]
fn marker_ok(t: &TransformStamped) -> bool {
    //
    let up = Vector3::unit_z();
    let q0 = Quaternion::new(t.transform.rotation.w, t.transform.rotation.x,
                             t.transform.rotation.y, t.transform.rotation.z);
    let rotated =  q0 * up;
    rotated.x.abs() < 0.2 && rotated.y.abs() < 0.2 && rotated.z.abs() > 0.9
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ros_ctx = Context::create()?;
    let mut node = Node::create(ros_ctx, "gantry_position_estimator", "")?;

    let sub = node.subscribe::<TransformStamped>("/aruco")?;
    let tf_pub = node.create_publisher::<TFMessage>("/rita/tf")?;
    let tf_pub2 = node.create_publisher::<TFMessage>("/tf")?;

    let mut trigger_srv = node.create_service::<Trigger::Service>("trigger")?;
    let ok_pub = node.create_publisher::<Bool>("measured")?;

    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;

    let state = Arc::new(Mutex::new(State::default()));

    let state_task = state.clone();
    let handle = tokio::task::spawn_blocking(move || loop {

        // check and remove stale transformations
        let now = clock.get_now().expect("could not get ros time");
        let time = r2r::Clock::to_builtin_time(&now);
        let sec = time.sec;

        {
            let mut state = state_task.lock().unwrap();
            if state.marker_0.as_ref().map(|t| (sec - t.header.stamp.sec) > 5).unwrap_or(false) {
                state.marker_0 = None;
                println!("stale marker 0, removing");
            }
            if state.marker_1.as_ref().map(|t| (sec - t.header.stamp.sec) > 5).unwrap_or(false) {
                state.marker_1 = None;
                println!("stale marker 1, removing");
            }
            if state.marker_2.as_ref().map(|t| (sec - t.header.stamp.sec) > 5).unwrap_or(false) {
                state.marker_2 = None;
                println!("stale marker 2, removing");
            }
            if state.marker_15.as_ref().map(|t| (sec - t.header.stamp.sec) > 5).unwrap_or(false) {
                state.marker_15 = None;
                println!("stale marker 15, removing");
            }
            if state.marker_5.as_ref().map(|t| (sec - t.header.stamp.sec) > 5).unwrap_or(false) {
                state.marker_5 = None;
                println!("stale marker 5, removing");
            }
        }

        // publish results.
        {
            let state = state_task.lock().unwrap();

            // publish floating positions to tf
            let mut transforms = vec![];
            if let Some(t) = state.facade_transform.as_ref() {
                transforms.push(t.clone());
            }
            if let Some(t) = state.gantry_transform.as_ref() {
                transforms.push(t.clone());
            }
            if let Some(t) = state.agv_transform.as_ref() {
                transforms.push(t.clone());
            }
            let tf_msg = TFMessage {
                transforms,
            };
            tf_pub.publish(&tf_msg).expect("could not publish");
            tf_pub2.publish(&tf_msg).expect("could not publish");

            // publish locked positions to tf.
            let mut transforms = vec![];
            if let Some(t) = state.locked_facade_transform.as_ref() {
                let mut t = t.clone();
                t.child_frame_id = "facade_locked".into();
                t.header.stamp = time.clone();
                transforms.push(t);
            }
            if let Some(t) = state.locked_gantry_transform.as_ref() {
                let mut t = t.clone();
                t.child_frame_id = "gantry_locked".into();
                t.header.stamp = time.clone();
                transforms.push(t);
            }
            let tf_msg = TFMessage {
                transforms,
            };
            tf_pub.publish(&tf_msg).expect("could not publish");
            tf_pub2.publish(&tf_msg).expect("could not publish");

            // publish to sp
            let ok = state.facade_transform.is_some() &&
                state.gantry_transform.is_some();
            let ok = Bool { data: ok };
            ok_pub.publish(&ok).expect("could not publish");
        }

        node.spin_once(std::time::Duration::from_millis(100));
    });


    let state_task = state.clone();
    tokio::spawn(async move {
        loop {
            if let Some(req) = trigger_srv.next().await {
                let mut state = state_task.lock().unwrap();
                state.locked_gantry_transform = state.gantry_transform.clone();
                state.locked_facade_transform = state.facade_transform.clone();

                let message = format!("gantry: {}, facade: {}",
                                      state.locked_gantry_transform.is_some(),
                                      state.locked_facade_transform.is_some(),
                );
                let response = Trigger::Response {
                    success: true,
                    message,
                };

                req.respond(response).expect("could not send response");
            }
        }
    });

    let interested_in = &["aruco_0", "aruco_1", "aruco_2", "aruco_15", "aruco_5"];
    sub.for_each(|msg| {
        if !interested_in.contains(&msg.child_frame_id.as_str()) {
            return future::ready(());
        }
        // println!("new msg: {:?}", msg);
        // if !marker_ok(&msg) {
        //     println!("bad marker: {}", msg.child_frame_id);
        //     return future::ready(());
        // }
        if msg.child_frame_id == "aruco_0" {
            update_or_set(msg.clone(), &mut state.lock().unwrap().marker_0);
        }
        if msg.child_frame_id == "aruco_1" {
            update_or_set(msg.clone(), &mut state.lock().unwrap().marker_1);
        }

        {
            let mut state = state.lock().unwrap();
            if state.marker_0.is_some() && state.marker_1.is_some() {
                let marker0 = state.marker_0.as_ref().unwrap().transform.clone();
                let marker1 = state.marker_1.as_ref().unwrap().transform.clone();

                let diff_x = marker1.translation.x - marker0.translation.x;
                let diff_y = marker1.translation.y - marker0.translation.y;
                let yaw = diff_y.atan2(diff_x);

                let mut new_transform = state.marker_1.as_ref().unwrap().clone();
                new_transform.child_frame_id = "facade_aruco".into();

                let rot = Quaternion::from(Euler {
                    x: Rad(0.0),
                    y: Rad(0.0),
                    z: Rad(yaw),
                });

                let rot2 = Quaternion::from(Euler {
                    x: Deg(180.0),
                    y: Deg(0.0),
                    z: Deg(0.0),
                });

                // set yaw and rotate around x to turn upside down.
                let new_q = rot * rot2;

                new_transform.transform.rotation.w = new_q.s;
                new_transform.transform.rotation.x = new_q.v.x;
                new_transform.transform.rotation.y = new_q.v.y;
                new_transform.transform.rotation.z = new_q.v.z;

                // set hardcoded height
                new_transform.transform.translation.z = 3.57;

                state.facade_transform = Some(new_transform);
            } else {
                state.facade_transform = None;
            }
        }

        if msg.child_frame_id == "aruco_2" {
            update_or_set(msg.clone(), &mut state.lock().unwrap().marker_2);
        }

        if msg.child_frame_id == "aruco_15" {
            update_or_set(msg.clone(), &mut state.lock().unwrap().marker_15);
        }

        {
            let mut state = state.lock().unwrap();
            if state.marker_15.is_some() && state.marker_2.is_some() {
                let marker15 = &state.marker_15.as_ref().unwrap().transform;
                let marker2 = &state.marker_2.as_ref().unwrap().transform;

                let diff_x = marker15.translation.x - marker2.translation.x;
                let diff_y = marker15.translation.y - marker2.translation.y;
                let yaw = diff_y.atan2(diff_x);

                // gantry position is marker15 position with this new rotation.
                let mut gantry_transform = state.marker_15.as_ref().unwrap().clone();
                gantry_transform.child_frame_id = "gantry_aruco".into();

                let rot = Quaternion::from(Euler {
                    x: Rad(0.0),
                    y: Rad(0.0),
                    z: Rad(yaw),
                });

                let rot2 = Quaternion::from(Euler {
                    x: Deg(180.0),
                    y: Deg(0.0),
                    z: Deg(0.0),
                });

                let gantry_q = rot * rot2;

                gantry_transform.transform.rotation.w = gantry_q.s;
                gantry_transform.transform.rotation.x = gantry_q.v.x;
                gantry_transform.transform.rotation.y = gantry_q.v.y;
                gantry_transform.transform.rotation.z = gantry_q.v.z;

                // hardcoded height
                gantry_transform.transform.translation.z = 1.93;

                state.gantry_transform = Some(gantry_transform);
            } else {
                state.gantry_transform = None;
            }
        }

        if msg.child_frame_id == "aruco_5" {
            update_or_set(msg.clone(), &mut state.lock().unwrap().marker_5);
        }

        {
            let mut state = state.lock().unwrap();
            if state.marker_5.is_some() {
                let mut agv_transform = state.marker_5.as_ref().unwrap().clone();
                agv_transform.child_frame_id = "agv_aruco".into();
                state.agv_transform = Some(agv_transform);
            }
        }

        future::ready(())
    }).await;

    handle.await?;

    Ok(())
}
