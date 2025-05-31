The best way to run async nodes in ROS 2 for high-performance robotics, like your Franka system, involves leveraging **multi-threaded executors** and **callback groups**. This allows sensor data, control loops, and data logging to operate concurrently without blocking each other.

---

## System and Design Principles for Asynchronous ROS 2 Applications

### Core Asynchronous Execution ⚙️

- **Principle 1: Employ Multi-Threaded Executors.**

  - Default to `rclpy.executors.MultiThreadedExecutor` (Python) or `rclcpp::executors::MultiThreadedExecutor` (C++) for all nodes that require concurrent task execution. This is the foundation for non-blocking behavior.

- **Principle 2: Strategically Assign Callbacks to Callback Groups.**
  - Use **`ReentrantCallbackGroup`** for tasks that can run in parallel without interference (e.g., most sensor data publishers, subscriber callbacks that queue data for an asynchronous writer).
  - Use **`MutuallyExclusiveCallbackGroup`** for critical sections or sequences that must not be interleaved (e.g., a robot control loop performing a read-process-command cycle).

---

### Node and Task Design 🧱

- **Principle 3: Isolate High-Frequency Tasks.**

  - Dedicate specific timers or nodes to individual high-frequency sensors (like cameras) to allow them to publish at their maximum possible rate.

- **Principle 4: Decouple I/O-Bound Operations.**

  - Implement tasks like MCAP writing asynchronously. A common pattern is a dedicated writer node/thread that consumes messages from a thread-safe internal queue, filled by lightweight subscriber callbacks. This prevents file I/O from blocking message processing or control loops.

- **Principle 5: Prioritize the Control Loop.**
  - Ensure the robot state acquisition and control command publication loop is highly responsive. Assign it to a suitable callback group (often mutually exclusive) and consider C++ for implementation if extreme low-latency is required.

---

### Data Handling and Communication 🔄

- **Principle 6: Optimize Quality of Service (QoS) Settings.**

  - For high-frequency sensor data where some loss is tolerable, use `Best Effort` reliability.
  - For critical messages like robot state and control commands, use `Reliable` reliability.
  - Keep history depth small (e.g., 1) for most high-frequency topics unless a longer history is explicitly needed.

- **Principle 7: Ensure Thread Safety.**
  - When using shared resources (like internal queues or shared state) between different callbacks or threads managed by the executor, always use appropriate synchronization primitives (mutexes, condition variables, thread-safe data structures).

---

### Performance and Implementation Choices 🚀

- **Principle 8: Select Language Based on Performance Needs.**

  - Use Python for rapid prototyping and less critical components.
  - Use C++ for performance-critical sections like control loops, complex sensor processing, or high-throughput data writers, especially to overcome Python's GIL limitations for CPU-bound tasks.

- **Principle 9: Leverage ROS 2 Performance Features.**

  - Utilize intra-process communication (IPC) by running related nodes in the same process where possible.
  - Consider loaned messages (C++) for zero-copy transport of large data types within a process.

- **Principle 10: Profile and Iterate.**
  - Regularly use profiling tools (`ros2_tracing`, `perf`) to identify bottlenecks in the system and refine the design or implementation accordingly.

---

### Franka-Specific Considerations 🦾

- **Principle 11: Maximize `franka_ros2` Capabilities.**
  - Thoroughly understand and utilize the topics, services, and controllers provided by the `franka_ros2` package for efficient robot state monitoring and command interfacing. For utmost control performance, evaluate if direct `libfranka` integration within a ROS 2 node is necessary.
