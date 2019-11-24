#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <applibs/log.h>
#include <applibs/gpio.h>

#include "ringbuf.h"

int main_buffer(void)
{
	RING_BUFFER test_buffer;
	float* test_data;
	// unsigned index;
	unsigned data_index;
	// unsigned count;
	// unsigned dummy;
	bool status;
	unsigned element_size;
	unsigned element_count;

	//float data_element16[16];
	float data_store[64];
	// testRingBuf(pTest, data_store, data_element,
	element_size = 1; //sizeof(data_element16),
	element_count = 64;
	// float* data_store = data_store16;
	// float* data_element = data_element16;

	Ringbuf_Init(&test_buffer, &data_store, element_size, element_count);

	/*for (data_index = 0; data_index < element_size; data_index++) {
		data_element[data_index] = *((float*)& data_index);  
	}*/

	float data = 1.1;
	status = Ringbuf_Put(&test_buffer, &data);
	data = 1.2;
	status = Ringbuf_Put(&test_buffer, &data);
	test_data = Ringbuf_Get_Front(&test_buffer);
	test_data = Ringbuf_Pop_Front(&test_buffer);
	test_data = Ringbuf_Pop_Front(&test_buffer);
    // This minimal Azure Sphere app repeatedly toggles GPIO 9, which is the green channel of RGB
    // LED 1 on the MT3620 RDB.
    // Use this app to test that device and SDK installation succeeded that you can build,
    // deploy, and debug an app with Visual Studio, and that you can deploy an app over the air,
    // per the instructions here: https://docs.microsoft.com/azure-sphere/quickstarts/qs-overview
    //
    // It is NOT recommended to use this as a starting point for developing apps; instead use
    // the extensible samples here: https://github.com/Azure/azure-sphere-samples
    Log_Debug(
        "\nVisit https://github.com/Azure/azure-sphere-samples for extensible samples to use as a "
        "starting point for full applications.\n");

    int fd = GPIO_OpenAsOutput(9, GPIO_OutputMode_PushPull, GPIO_Value_High);
    if (fd < 0) {
        Log_Debug(
            "Error opening GPIO: %s (%d). Check that app_manifest.json includes the GPIO used.\n",
            strerror(errno), errno);
        return -1;
    }

    const struct timespec sleepTime = {1, 0};
    while (true) {
        GPIO_SetValue(fd, GPIO_Value_Low);
        nanosleep(&sleepTime, NULL);
        GPIO_SetValue(fd, GPIO_Value_High);
        nanosleep(&sleepTime, NULL);
    }
}