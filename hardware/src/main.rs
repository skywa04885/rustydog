use tokio::signal::ctrl_c;
use tokio::task;
use tokio_util::sync::CancellationToken;
use tokio_util::task::TaskTracker;
use hardware::{Hardware, Servo, ServoTarget};

#[tokio::main]
pub async fn main() {
    let (mut task, handle) = Hardware::new("hardware", "localhost", 1883, Some(("arduino", "Ffeirluke234")));

    let cancellation_token = CancellationToken::new();
    let task_tracker = TaskTracker::new();

    task_tracker.spawn({
        let cancellation_token = cancellation_token.clone();

        async move {
            task.run(cancellation_token).await;
        }
    });

    handle.push_into_buffer(Servo::Joint0, vec![
        ServoTarget::new(120, 200),
        ServoTarget::new(0, 200),
        ServoTarget::new(120, 200),
        ServoTarget::new(0, 200),
        ServoTarget::new(120, 200),
        ServoTarget::new(0, 200),
        ServoTarget::new(120, 200),
    ],cancellation_token.clone()).await.unwrap();

    println!("Ready!");

    ctrl_c().await.unwrap();

    cancellation_token.cancel();

    task_tracker.close();
    task_tracker.wait().await;
}