use regex::Regex;
use rumqttc::{AsyncClient, Event, EventLoop, MqttOptions, Packet, QoS};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::error::Error;
use std::future::Future;
use std::hash::Hash;
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::Ordering::SeqCst;
use std::sync::Arc;
use std::time::Duration;
use strum::IntoEnumIterator;
use strum_macros::EnumIter;
use thiserror::Error;
use tokio::select;
use tokio::sync::broadcast::Receiver as BroadcastReceiver;
use tokio::sync::oneshot::{channel as oneshotChannel, Sender as OneshotSender};
use tokio::sync::{broadcast, Mutex};
use tokio::time::{sleep, timeout};
use tokio_stream::StreamExt;
use tokio_util::sync::CancellationToken;

#[derive(Copy, Clone, EnumIter, PartialEq, Eq, Debug, Hash)]
pub enum Servo {
    Joint0,
    Joint1,
    Joint2,
    Joint3,
    Joint4,
}

impl Servo {
    pub fn identifier(&self) -> i16 {
        match self {
            &Servo::Joint0 => 0,
            &Servo::Joint1 => 1,
            &Servo::Joint2 => 2,
            &Servo::Joint3 => 3,
            &Servo::Joint4 => 4,
        }
    }
}

#[derive(Serialize, Deserialize, Copy, Clone)]
pub struct ServoTarget {
    angle: i16,
    speed: i16,
}

impl ServoTarget {
    pub fn new(angle: i16, speed: i16) -> Self {
        Self { angle, speed }
    }
}

#[derive(Debug, Clone)]
pub enum ServoBufferEvent {
    Drain(i16),
    Empty,
}

pub struct ServoBuffer {
    event_tx: broadcast::Sender<ServoBufferEvent>,
}

impl ServoBuffer {
    fn new() -> Self {
        let event_tx = broadcast::Sender::<ServoBufferEvent>::new(64);

        Self { event_tx }
    }

    pub async fn drain(
        &self,
        cancellation_token: CancellationToken,
    ) -> Result<i16, Box<dyn Error>> {
        let mut receiver: BroadcastReceiver<ServoBufferEvent> = self.event_tx.subscribe();

        loop {
            if let ServoBufferEvent::Drain(need) = select! {
                event = receiver.recv() => event?,
                _ = cancellation_token.cancelled() => return Err(Box::new(HardwareTaskError::Cancelled)),
            } {
                return Ok(need);
            }
        }
    }

    pub async fn empty(&self, cancellation_token: CancellationToken) -> Result<(), Box<dyn Error>> {
        let mut receiver: BroadcastReceiver<ServoBufferEvent> = self.event_tx.subscribe();

        loop {
            if let ServoBufferEvent::Empty = select! {
                event = receiver.recv() => event?,
                _ = cancellation_token.cancelled() => return Err(Box::new(HardwareTaskError::Cancelled)),
            } {
                return Ok(());
            }
        }
    }
}

/// The message for the get angle request.
#[derive(Serialize)]
pub struct ServoGetAngleMessageRequest {
    response_topic: String,
}

impl ServoGetAngleMessageRequest {
    pub fn new(response_topic: String) -> Self {
        Self { response_topic }
    }
}

/// The message for the get angle response.
#[derive(Serialize)]
pub struct ServoGetAngleMessageResponse {
    angle: i16,
}

impl ServoGetAngleMessageResponse {
    pub fn new(angle: i16) -> Self {
        Self { angle }
    }
}

/// The message for the get buffer capacity request.
#[derive(Serialize)]
pub struct BufferGetCapacityMessageRequest {
    response_topic: String,
}

impl BufferGetCapacityMessageRequest {
    pub fn new(response_topic: String) -> Self {
        Self { response_topic }
    }
}

/// The message for the get capacity response.
#[derive(Deserialize)]
pub struct BufferGetCapacityMessageResponse {
    capacity: i16,
}

impl BufferGetCapacityMessageResponse {
    pub fn new(capacity: i16) -> Self {
        Self { capacity }
    }
}

#[derive(Serialize)]
pub struct BufferPushMessage {
    targets: Vec<ServoTarget>,
}

impl BufferPushMessage {
    pub fn new<S: Into<Vec<ServoTarget>>>(targets: S) -> Self {
        Self {
            targets: targets.into(),
        }
    }
}

/// Generate the buffer push topic string for the given servo.
pub fn buffer_push_message_topic(servo: Servo) -> String {
    format!("/nodes/arduino/servo/{}/buffer/push", servo.identifier())
}

#[derive(Serialize, Deserialize)]
pub struct BufferDrainMessage {
    pub need: i16,
}

impl BufferDrainMessage {}

/// Generate the buffer drain topic string for the given servo.
pub fn servo_buffer_drain_topic(servo: Servo) -> String {
    format!("/nodes/arduino/servo/{}/buffer/drain", servo.identifier())
}

#[derive(Serialize, Deserialize)]
pub struct BufferEmptyMessage {}

impl BufferEmptyMessage {}

/// Generate the buffer empty topic string for the given servo.
pub fn servo_buffer_empty_topic(servo: Servo) -> String {
    format!("/nodes/arduino/servo/{}/buffer/empty", servo.identifier())
}

pub enum HardwareCommand {}

pub struct Hardware;

impl Hardware {
    /// Create a new hardware task instance with the given options.
    pub fn new<S: Into<String>, T: Into<String>, U: Into<String>, V: Into<String>>(
        id: S,
        host: T,
        port: u16,
        credentials: Option<(U, V)>,
    ) -> (HardwareTask, HardwareHandle) {
        // Create the mqtt options.
        let mut mqtt_options = MqttOptions::new(id, host, port);
        mqtt_options.set_keep_alive(Duration::from_secs(5));

        // Add the credentials to the options, if they've been given.
        if let Some((username, password)) = credentials {
            mqtt_options.set_credentials(username, password);
        }

        // Create the client with the options.
        let (client, mut ev_loop) = AsyncClient::new(mqtt_options, 100_usize);

        // Create the buffer registry.
        let buffer_registry: HardwareBufferRegistry = HardwareBufferRegistry::new();

        // Construct the hardware task and it's handle.
        let hardware_task: HardwareTask =
            HardwareTask::new(ev_loop, client.clone(), buffer_registry.clone());
        let hardware_handle: HardwareHandle = HardwareHandle::new(client, buffer_registry);

        // Return the hardware task and handle.
        (hardware_task, hardware_handle)
    }
}

#[derive(Clone)]
pub struct HardwareBufferRegistry {
    inner: Arc<Mutex<HashMap<Servo, Arc<ServoBuffer>>>>,
}

impl HardwareBufferRegistry {
    pub fn new() -> Self {
        let mut inner = HashMap::<Servo, Arc<ServoBuffer>>::new();

        for servo in Servo::iter() {
            inner.insert(servo, Arc::new(ServoBuffer::new()));
        }

        Self {
            inner: Arc::new(Mutex::new(inner)),
        }
    }

    pub async fn get_for_servo(&self, servo: Servo) -> Result<Arc<ServoBuffer>, HardwareTaskError> {
        if let Some(buffer_ptr) = self.inner.lock().await.get(&servo) {
            Ok(buffer_ptr.clone())
        } else {
            Err(HardwareTaskError::ServoNotInRegistry(servo))
        }
    }
}

pub struct HardwareTask {
    ev_loop: EventLoop,
    client: AsyncClient,
    response_counter: Arc<AtomicUsize>,
    response_handlers: Arc<Mutex<HashMap<usize, OneshotSender<String>>>>,
    buffer_registry: HardwareBufferRegistry,
}

#[derive(Error, Debug)]
pub enum HardwareTaskError {
    #[error("Operation cancelled")]
    Cancelled,
    #[error("Servo not in registry {0:?}")]
    ServoNotInRegistry(Servo),
    #[error("Invalid servo index {0}")]
    InvalidServoIndex(i16),
}

impl HardwareTask {
    fn new(
        ev_loop: EventLoop,
        client: AsyncClient,
        buffer_registry: HardwareBufferRegistry,
    ) -> Self {
        let response_handlers = Arc::new(Mutex::new(HashMap::new()));
        let response_counter = Arc::new(AtomicUsize::new(0_usize));

        Self {
            ev_loop,
            client,
            response_counter,
            response_handlers,
            buffer_registry,
        }
    }

    pub async fn register_future_response(
        &mut self,
        timeout_after: Duration,
        cancellation_token: CancellationToken,
    ) -> Result<
        (
            String,
            Box<dyn Future<Output = Result<Option<String>, Box<dyn Error>>> + '_>,
        ),
        Box<dyn Error>,
    > {
        // Get the response count and generate the response topic.
        let response_count: usize = self.response_counter.fetch_add(1_usize, SeqCst);
        let topic: String = format!("/nodes/hardware/response/{}", response_count);

        // Subscribe to the topic.
        self.client
            .subscribe(topic.clone(), QoS::ExactlyOnce)
            .await?;

        // Create the oneshot channel.
        let (sender, receiver) = oneshotChannel::<String>();

        // Insert the sender into the response handlers map.
        self.response_handlers
            .lock()
            .await
            .insert(response_count, sender);

        // Return the topic and the receiver.
        Ok((
            topic.clone(),
            Box::new(async move {
                let timeout_future = timeout(timeout_after, receiver);
                let cancellation_future = cancellation_token.cancelled();

                let result: Result<Option<String>, Box<dyn Error>> = select! {
                    _ = cancellation_future => {
                        // Remove the sender from the response handlers map.
                        self.response_handlers.lock().await.remove(&response_count);

                        // Return the error that indicates it was cancelled.
                        Err(Box::new(HardwareTaskError::Cancelled))
                    },
                    timeout_result = timeout_future => {
                        if let Ok(rx_result) = timeout_result {
                            // Return the received message.
                            Ok(Some(rx_result?))
                        } else {
                            // Remove the sender from the response handlers map.
                            self.response_handlers.lock().await.remove(&response_count);

                            // Return none since we timed out.
                            Ok(None)
                        }
                    },
                };

                // Unsubscribe to the topic.
                self.client.unsubscribe(topic).await?;

                result
            }),
        ))
    }

    pub async fn run(
        &mut self,
        cancellation_token: CancellationToken,
    ) -> Result<(), Box<dyn Error>> {
        // Subscribe to the topics which contain buffer events.
        self.client
            .subscribe("/nodes/arduino/servo/+/buffer/empty", QoS::ExactlyOnce)
            .await?;
        self.client
            .subscribe("/nodes/arduino/servo/+/buffer/drain", QoS::ExactlyOnce)
            .await?;

        let buffer_empty_regex =
            Regex::new(r"/nodes/arduino/servo/(?<n>[0-5])/buffer/empty").unwrap();
        let buffer_drain_regex =
            Regex::new(r"/nodes/arduino/servo/(?<n>[0-5])/buffer/drain").unwrap();

        // Process all the events.
        loop {
            let event = select! {
                event = self.ev_loop.poll() => event?,
                _ = cancellation_token.cancelled() => return Err(Box::new(HardwareTaskError::Cancelled)),
            };

            match event {
                Event::Incoming(Packet::Publish(publish)) => {
                    if let Some(captures) = buffer_empty_regex.captures(&publish.topic) {
                        let n: i16 = captures["n"].parse()?;
                        let message: BufferEmptyMessage =
                            serde_json::from_str(std::str::from_utf8(&publish.payload)?)?;

                        // Get the servo with the given index.
                        let servo = Servo::iter()
                            .nth(n as usize)
                            .ok_or(Box::new(HardwareTaskError::InvalidServoIndex(n)))?;

                        // Get the buffer from the registry.
                        let buffer = self.buffer_registry.get_for_servo(servo).await?;

                        // Send the event.
                        buffer.event_tx.send(ServoBufferEvent::Empty)?;
                    } else if let Some(captures) = buffer_drain_regex.captures(&publish.topic) {
                        let n: i16 = captures["n"].parse()?;
                        let message: BufferDrainMessage =
                            serde_json::from_str(std::str::from_utf8(&publish.payload)?)?;

                        // Get the servo with the given index.
                        let servo = Servo::iter()
                            .nth(n as usize)
                            .ok_or(Box::new(HardwareTaskError::InvalidServoIndex(n)))?;

                        // Get the buffer from the registry.
                        let buffer = self.buffer_registry.get_for_servo(servo).await?;

                        // Send the event.
                        buffer
                            .event_tx
                            .send(ServoBufferEvent::Drain(message.need))?;
                    }
                }
                _ => {}
            }
        }
    }
}

pub struct HardwareHandle {
    client: AsyncClient,
    buffer_registry: HardwareBufferRegistry,
}

impl HardwareHandle {
    const BUFFER_CAPACITY: i16 = 20_i16;

    fn new(client: AsyncClient, buffer_registry: HardwareBufferRegistry) -> Self {
        Self {
            client,
            buffer_registry,
        }
    }

    pub async fn buffer_push_message(
        &self,
        servo: Servo,
        message: BufferPushMessage,
    ) -> Result<(), Box<dyn Error>> {
        let topic: String = buffer_push_message_topic(servo);
        let message: String = serde_json::to_string(&message)?;

        self.client
            .publish(topic, QoS::ExactlyOnce, false, message)
            .await?;

        Ok(())
    }

    pub async fn push_into_buffer<S: Into<Vec<ServoTarget>>>(
        &self,
        servo: Servo,
        targets: S,
        cancellation_token: CancellationToken,
    ) -> Result<(), Box<dyn Error>> {
        let buffer: Arc<ServoBuffer> = self.buffer_registry.get_for_servo(servo).await?;

        let targets: Vec<ServoTarget> = targets.into();

        let mut i: i16 = 0;
        let mut need: i16 = Self::BUFFER_CAPACITY;

        loop {
            // Compute how many targets to consume.
            let consume: i16 = need.min(targets.len() as i16 - i);

            // Send the buffer push message.
            self.buffer_push_message(
                servo,
                BufferPushMessage::new(&targets[i as usize..(i + consume) as usize]),
            )
            .await?;

            // Add the consumed targets to the iterator, and break if there's nothing left.
            i += consume;
            if i == targets.len() as i16 {
                break;
            }

            // Wait for the drain condition, and adjust the need variable to contain the number of
            //  targets which can be pushed to the buffer.
            need = buffer.drain(cancellation_token.clone()).await?;
        }

        // Wait for the buffer empty event, then we can finish the current movement.
        buffer.empty(cancellation_token.clone()).await?;

        Ok(())
    }
}
