# HealthStatus message for Physical AI System Monitoring
# Represents the current health metrics of a system component

std_msgs/Header header
string node_name
float64 timestamp
float64 cpu_percent
float64 memory_percent
float64 disk_percent
float64 temperature
string status
