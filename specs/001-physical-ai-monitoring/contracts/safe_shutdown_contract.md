# Safe Shutdown Service Contract

## Overview
The `/safe_shutdown` service provides a way to initiate safe shutdown procedures for Physical AI hardware components when critical conditions are detected.

## Service Definition
```
# SafeShutdown.srv
# Request
string reason
string requested_by
float64 timeout

# Response
bool success
string message
```

## Request Fields

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| reason | string | Reason for the shutdown request | Required, non-empty |
| requested_by | string | Node or service requesting shutdown | Required, non-empty |
| timeout | float64 | Maximum time allowed for shutdown (seconds) | > 0.0, ≤ 300.0 (5 minutes) |

## Response Fields

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| success | bool | Whether shutdown was initiated successfully | true/false |
| message | string | Human-readable status message | Non-empty on error |

## Example Request
```
reason: "OVERTEMP"
requested_by: "temperature_monitor"
timeout: 30.0
```

## Example Response
```
success: true
message: "Safe shutdown initiated successfully"
```

## Service Interface
- **Service Name**: `/safe_shutdown`
- **Service Type**: `edge_health_interfaces/srv/SafeShutdown`
- **Service Type**: Synchronous (request/response)
- **Timeout**: Service should respond within 5 seconds

## Valid Reasons
- `OVERTEMP`: Over-temperature condition detected
- `CRITICAL_ALERT`: Critical system alert triggered
- `EMERGENCY`: Emergency shutdown required
- `MAINTENANCE`: Scheduled maintenance shutdown
- `POWER_LOW`: Low power condition
- `USER_REQUEST`: Manual shutdown request

## Implementation Requirements

### Pre-shutdown Checks
1. Verify the requesting node has permission to initiate shutdown
2. Validate the timeout value is within acceptable range
3. Check that no other shutdown is currently in progress
4. Log the shutdown request with details

### Shutdown Sequence
1. Publish shutdown initiation message to `/shutdown_status` topic
2. Notify all dependent services of impending shutdown
3. Execute hardware-specific safe shutdown procedures
4. Power down components in proper sequence
5. Return success response when complete

### Error Conditions
- `shutdown_in_progress`: Another shutdown is already in progress
- `invalid_timeout`: Timeout value is outside acceptable range
- `unauthorized`: Requesting node is not authorized
- `hardware_error`: Hardware error during shutdown procedure
- `timeout`: Shutdown procedure exceeded specified timeout

## Security Considerations
- Only authorized nodes should be allowed to call this service
- Reason validation to prevent invalid shutdown requests
- Rate limiting to prevent shutdown flooding
- Logging of all shutdown requests for audit trail

## Expected Callers
- Alert processing nodes
- Temperature monitoring systems
- Operator interfaces
- Automated response systems

## Error Response Examples
```
# Shutdown already in progress
success: false
message: "Shutdown already in progress"

# Invalid timeout value
success: false
message: "Timeout value 400.0 exceeds maximum of 300.0 seconds"

# Unauthorized caller
success: false
message: "Node 'unauthorized_node' is not authorized to request shutdown"
```