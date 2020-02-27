#ifndef UCCN_CONFIG_H_
#define UCCN_CONFIG_H_

#ifndef CONFIG_UCCN_PORT
#define CONFIG_UCCN_PORT 15243
#endif

#ifndef CONFIG_UCCN_MAX_NODE_NAME_SIZE
#define CONFIG_UCCN_MAX_NODE_NAME_SIZE 64
#endif

#ifndef CONFIG_UCCN_MAX_RESOURCE_PATH_SIZE
#define CONFIG_UCCN_MAX_RESOURCE_PATH_SIZE 64
#endif

#ifndef CONFIG_UCCN_MAX_NUM_PEERS
#define CONFIG_UCCN_MAX_NUM_PEERS 5
#endif

#ifndef CONFIG_UCCN_MAX_NUM_RESOURCES
#define CONFIG_UCCN_MAX_NUM_RESOURCES 10
#endif

#ifndef CONFIG_UCCN_MAX_NUM_TRACKERS
#define CONFIG_UCCN_MAX_NUM_TRACKERS CONFIG_UCCN_MAX_NUM_RESOURCES
#endif

#if CONFIG_UCCN_MAX_NUM_TRACKERS > CONFIG_UCCN_MAX_NUM_RESOURCES
#error "uCCN cannot track more resources than available"
#endif

#ifndef CONFIG_UCCN_MAX_NUM_PROVIDERS
#define CONFIG_UCCN_MAX_NUM_PROVIDERS CONFIG_UCCN_MAX_NUM_RESOURCES
#endif

#if CONFIG_UCCN_MAX_NUM_TRACKERS > CONFIG_UCCN_MAX_NUM_RESOURCES
#error "uCCN cannot provide more resources than available"
#endif

#ifndef CONFIG_UCCN_MAX_CONTENT_SIZE
#define CONFIG_UCCN_MAX_CONTENT_SIZE 256
#endif

#ifndef CONFIG_UCCN_INCOMING_BUFFER_SIZE
#define CONFIG_UCCN_INCOMING_BUFFER_SIZE 512
#endif

#if CONFIG_UCCN_MAX_CONTENT_SIZE > CONFIG_UCCN_INCOMING_BUFFER_SIZE
#error "uCCN incoming buffer cannot be smaller than the maximum content size"
#endif

#ifndef CONFIG_UCCN_OUTGOING_BUFFER_SIZE
#define CONFIG_UCCN_OUTGOING_BUFFER_SIZE 512
#endif

#if CONFIG_UCCN_MAX_CONTENT_SIZE > CONFIG_UCCN_OUTGOING_BUFFER_SIZE
#error "uCCN outgoing buffer cannot be smaller than the maximum content size"
#endif

#ifndef CONFIG_UCCN_LIVELINESS_TIMEOUT_MS
#define CONFIG_UCCN_LIVELINESS_TIMEOUT_MS 2000
#endif

#ifndef CONFIG_UCCN_LIVELINESS_ASSERT_TIMEOUT_MS
#define CONFIG_UCCN_LIVELINESS_ASSERT_TIMEOUT_MS 1000
#endif

#if CONFIG_UCCN_LIVELINESS_ASSERT_TIMEOUT_MS >= CONFIG_UCCN_LIVELINESS_TIMEOUT_MS
#error "uCCN liveliness assertions must be sent before it times out"
#endif

#ifndef CONFIG_UCCN_PEER_DISCOVERY_PERIOD_MS
#define CONFIG_UCCN_PEER_DISCOVERY_PERIOD_MS 1000
#endif

#ifndef CONFIG_UCCN_ENDPOINT_PROBE_TIMEOUT_MS
#define CONFIG_UCCN_ENDPOINT_PROBE_TIMEOUT_MS 500
#endif

#if CONFIG_UCCN_ENDPOINT_PROBE_TIMEOUT_MS >= CONFIG_UCCN_PEER_DISCOVERY_PERIOD_MS
#error "uCCN must probe for endpoint state faster than it discovers peers"
#endif

#ifndef CONFIG_UCCN_MULTITHREADED
#define CONFIG_UCCN_MULTITHREADED 1
#endif

#ifndef CONFIG_UCCN_LOGGING
#define CONFIG_UCCN_LOGGING 1
#endif

#endif  // UCCN_CONFIG_H_
