#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define STRING_BUFFER_LEN 100

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


const uint LED_PIN = 25;

rcl_publisher_t publisher;
rcl_subscription_t ping_subscriber;

std_msgs__msg__Int32 msg;
geometry_msgs__msg__Twist incoming_ping;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}



void ping_subscription_callback(const void * msgin)
{
	const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

	// Dont pong my own pings
    /*
	if(strcmp(outcoming_ping.frame_id.data, msg->frame_id.data) != 0){
		printf("Ping received with seq %s. Answering.\n", msg->frame_id.data);
		rcl_publish(&pong_publisher, (const void*)msg, NULL);
	}
    */
   printf("Msg received with x=%i,y=%i,z=%i", msg->linear.x, msg->linear.y, msg->linear.z);
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);


  
    rcl_allocator_t allocator;
    rclc_support_t support;
  

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "pico_node", "", &support));

    // Create a reliable ping publisher
    RCCHECK( rclc_publisher_init_default(&publisher, &node,  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32) , "pico_publisher"));

    // Create a best effort subscriber
    RCCHECK( rclc_subscription_init_best_effort( &ping_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/pose"))
    
    // Create a 1 second ping timer timer
    rcl_timer_t timer;
    RCCHECK( rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), timer_callback) );

    // Create executor
    rclc_executor_t executor;
    RCCHECK( rclc_executor_init(&executor, &support.context, 1, &allocator) );
    RCCHECK( rclc_executor_add_timer(&executor, &timer) );
    RCCHECK( rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, &ping_subscription_callback, ON_NEW_DATA));

    // Create and allocate the messages

	char incoming_ping_buffer[STRING_BUFFER_LEN];
	
    gpio_put(LED_PIN, 1);

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
