# Hardware

This section covers hardware integration guides for various platforms supported by the Swarm Manager system.

## Supported Platforms

The system supports multiple hardware platforms for different use cases:

- **[ESP32](esp32.md)** - Low-cost WiFi/Bluetooth microcontroller for lightweight drones
- **[PX4](px4.md)** - Professional autopilot platform for advanced drone operations
- **[STM32](stm32.md)** - High-performance microcontroller for custom implementations

## Platform Comparison

| Platform | Use Case | Communication | Processing Power |
|----------|----------|---------------|------------------|
| ESP32 | Lightweight swarms | WiFi/BLE | Moderate |
| PX4 | Professional UAVs | MAVLink | High |
| STM32 | Custom hardware | Flexible | High |

## Quick Links

<div class="grid cards" markdown>

-   :material-wifi: **ESP32 Setup**

    ---

    Configure ESP32-based drones for swarm communication

    [:octicons-arrow-right-24: ESP32 guide](esp32.md)

-   :material-airplane: **PX4 Integration**

    ---

    Integrate with PX4-based autopilot systems

    [:octicons-arrow-right-24: PX4 guide](px4.md)

-   :material-chip: **STM32 Support**

    ---

    Set up STM32 microcontrollers for custom builds

    [:octicons-arrow-right-24: STM32 guide](stm32.md)

</div>
