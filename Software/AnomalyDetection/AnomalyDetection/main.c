/* Copyright (c) Ron Dagdag */

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>

#include <string.h> 
#include <curl/curl.h>
#include <math.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/storage.h>

#include "epoll_timerfd_utilities.h"
#include "i2c.h"
#include "lsm6dso_reg.h"
#include "lps22hh_reg.h"

#include "build_options.h"
// #include "ringbuf.h"

#include "lcthw/list.h"
#include "lcthw/list_algos.h"
#include "id.h"

#include "relay.h"
#include "applibs_versions.h"
#include <applibs/log.h>

#include "mt3620_rdb.h"

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t raw_angular_rate_calibration;
static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_dps[3];
static float lsm6dsoTemperature_degC;
static float pressure_hPa;
static float lps22hhTemperature_degC;
static float tracked_value;

RELAY* rptr;

#define JSONDATA_BUFFER_SIZE 8092

typedef struct Entry {
	double data; /* block of memory or array of data */
	char* time;
} Entry;
#define TIMESIZE 128
#define KEYSIZE 128
List* entries;
//RING_BUFFER data_buffer;
float* last_data;
bool status;

element_size = 1; //sizeof(data_element16),
element_count = 1000;
float data_store[1000];

static uint8_t whoamI, rst;
static int accelTimerFd = -1;
// const uint8_t lsm6dsOAddress = LSM6DSO_ADDRESS;     // Addr = 0x6A
lsm6dso_ctx_t dev_ctx;
lps22hh_ctx_t pressure_ctx;

JsonFrame_TypeDef outgoingFrame;

// Termination state
volatile sig_atomic_t terminationRequired = false;

static const struct timespec LoopInterval = { 0, 200000000 };

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

// Epoll and event handler file descriptors.
static int webpageDownloadTimerFd = -1;
static int sensorTimerFd = -1;
static int listTimerFd = -1;
int epollFd = -1;

/// <summary>
///     Data pointer and size of a block of memory allocated on the heap.
/// </summary>
typedef struct {
    char *data;
    size_t size;
} MemoryBlock;

/// <summary>
///     Callback for curl_easy_perform() that copies all the downloaded chunks in a single memory
///     block.
/// <param name="chunks">The pointer to the chunks array</param>
/// <param name="chunkSize">The size of each chunk</param>
/// <param name="chunksCount">The count of the chunks</param>
/// <param name="memoryBlock">The pointer where all the downloaded chunks are aggregated</param>
/// </summary>
static size_t StoreDownloadedDataCallback(void *chunks, size_t chunkSize, size_t chunksCount,
                                          void *memoryBlock)
{
    MemoryBlock *block = (MemoryBlock *)memoryBlock;

    size_t additionalDataSize = chunkSize * chunksCount;
    block->data = realloc(block->data, block->size + additionalDataSize + 1);
    if (block->data == NULL) {
        Log_Debug("Out of memory, realloc returned NULL: errno=%d (%s)'n", errno, strerror(errno));
        abort();
    }

    memcpy(block->data + block->size, chunks, additionalDataSize);
    block->size += additionalDataSize;
    block->data[block->size] = 0; // Ensure the block of memory is null terminated.

    return additionalDataSize;
}

/// <summary>
///     Logs a cURL error.
/// </summary>
/// <param name="message">The message to print</param>
/// <param name="curlErrCode">The cURL error code to describe</param>
static void LogCurlError(const char *message, int curlErrCode)
{
    Log_Debug(message);
    Log_Debug(" (curl err=%d, '%s')\n", curlErrCode, curl_easy_strerror(curlErrCode));
}

/// <summary>
///     Download a web page over HTTPS protocol using cURL.
/// </summary>
static void PerformWebPageDownload(void)
{
    CURL *curlHandle = NULL;
    CURLcode res = 0;
    MemoryBlock block = {.data = NULL, .size = 0};
    char *certificatePath = NULL;

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) < 0) || !isNetworkingReady) {
        Log_Debug("\nNot doing download because network is not up.\n");
        goto exitLabel;
    }

    Log_Debug("\n -===- Starting download -===-\n");

    // Init the cURL library.
    if ((res = curl_global_init(CURL_GLOBAL_ALL)) != CURLE_OK) {
        LogCurlError("curl_global_init", res);
        goto exitLabel;
    }

    if ((curlHandle = curl_easy_init()) == NULL) {
        Log_Debug("curl_easy_init() failed\n");
        goto cleanupLabel;
    }

    // Specify URL to download.
    // Important: any change in the domain name must be reflected in the AllowedConnections
    // capability in app_manifest.json.
    /*if ((res = curl_easy_setopt(curlHandle, CURLOPT_URL, "http://192.168.43.94:5000/status")) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_URL", res);
        goto cleanupLabel;
    }*/
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_URL, "http://192.168.1.18:5000/status")) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_URL", res);
		goto cleanupLabel;
	}

    // Set output level to verbose.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_VERBOSE, 1L)) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_VERBOSE", res);
        goto cleanupLabel;
    }

    // Get the full path to the certificate file used to authenticate the HTTPS server identity.
    // The DigiCertGlobalRootCA.pem file is the certificate that is used to verify the
    // server identity.
    certificatePath = Storage_GetAbsolutePathInImagePackage("certs/DigiCertGlobalRootCA.pem");
    if (certificatePath == NULL) {
        Log_Debug("The certificate path could not be resolved: errno=%d (%s)\n", errno,
                  strerror(errno));
        goto cleanupLabel;
    }

    // Set the path for the certificate file that cURL uses to validate the server certificate.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_CAINFO, certificatePath)) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_CAINFO", res);
        goto cleanupLabel;
    }

    // Let cURL follow any HTTP 3xx redirects.
    // Important: any redirection to different domain names requires that domain name to be added to
    // app_manifest.json.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_FOLLOWLOCATION, 1L)) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_FOLLOWLOCATION", res);
        goto cleanupLabel;
    }

    // Set up callback for cURL to use when downloading data.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, StoreDownloadedDataCallback)) !=
        CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_FOLLOWLOCATION", res);
        goto cleanupLabel;
    }

    // Set the custom parameter of the callback to the memory block.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_WRITEDATA, (void *)&block)) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_WRITEDATA", res);
        goto cleanupLabel;
    }

    // Specify a user agent.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_USERAGENT, "libcurl-agent/1.0")) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_USERAGENT", res);
        goto cleanupLabel;
    }

    // Perform the download of the web page.
    if ((res = curl_easy_perform(curlHandle)) != CURLE_OK) {
        LogCurlError("curl_easy_perform", res);
    } else {
        Log_Debug("\n -===- Downloaded content (%zu bytes): -===-\n", block.size);
        Log_Debug("%s\n", block.data);
    }

cleanupLabel:
    // Clean up allocated memory.
    free(block.data);
    free(certificatePath);
    // Clean up sample's cURL resources.
    curl_easy_cleanup(curlHandle);
    // Clean up cURL library's resources.
    curl_global_cleanup();
    Log_Debug("\n -===- End of download -===-\n");

exitLabel:
    return;
}


static void PerformWebhookPOST(char* jsonString)
{
	CURL* curlHandle = NULL;
	CURLcode res = 0;
	MemoryBlock block = { .data = NULL,.size = 0 };
	char* certificatePath = NULL;

	bool isNetworkingReady = false;
	if ((Networking_IsNetworkingReady(&isNetworkingReady) < 0) || !isNetworkingReady) {
		Log_Debug("\nNot doing download because network is not up.\n");
		goto exitLabel;
	}

	Log_Debug("\n -===- Starting POST -===-\n");

	// Init the cURL library.
	if ((res = curl_global_init(CURL_GLOBAL_ALL)) != CURLE_OK) {
		LogCurlError("curl_global_init", res);
		goto exitLabel;
	}

	if ((curlHandle = curl_easy_init()) == NULL) {
		Log_Debug("curl_easy_init() failed\n");
		goto cleanupLabel;
	}

	// Specify URL to POST to.
	// Important: any change in the domain name must be reflected in the AllowedConnections
	// capability in app_manifest.json.
	// char* url = "https://anomaly-detector-ron.cognitiveservices.azure.com/anomalydetector/v1.0/timeseries/last/detect";
	//char* url = "http://192.168.43.94:5000/anomalydetector/v1.0/timeseries/last/detect";
	char* url = "http://192.168.1.18:5000/anomalydetector/v1.0/timeseries/last/detect";

	if ((res = curl_easy_setopt(curlHandle, CURLOPT_URL, url)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_URL", res);
		goto cleanupLabel;
	}

	// Set output level to verbose.
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_VERBOSE, 1L)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_VERBOSE", res);
		goto cleanupLabel;
	}

	// Get the full path to the certificate file used to authenticate the HTTPS server identity.
	// The DigiCertGlobalRootCA.pem file is the certificate that is used to verify the
	// server identity.
	certificatePath = Storage_GetAbsolutePathInImagePackage("certs/DigiCertGlobalRootCA.pem");
	if (certificatePath == NULL) {
		Log_Debug("The certificate path could not be resolved: errno=%d (%s)\n", errno,
			strerror(errno));
		goto cleanupLabel;
	}

	// Set the path for the certificate file that cURL uses to validate the server certificate.
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_CAINFO, certificatePath)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_CAINFO", res);
		goto cleanupLabel;
	}

	if ((res = curl_easy_setopt(curlHandle, CURLOPT_POST, 1L)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_POST", res);
		goto cleanupLabel;
	}


	if ((res = curl_easy_setopt(curlHandle, CURLOPT_POSTFIELDS, jsonString)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_POSTFIELDS", res);
		goto cleanupLabel;
	}

	struct curl_slist* hs = NULL;
	hs = curl_slist_append(hs, "Content-Type: application/json");
	char* subkey = (char*)malloc(KEYSIZE);
	//sprintf(subkey, "Ocp-Apim-Subscription-Key: %s", key);
	//hs = curl_slist_append(hs, subkey);
	curl_easy_setopt(curlHandle, CURLOPT_HTTPHEADER, hs);

	// Set up callback for cURL to use when downloading data.
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_WRITEFUNCTION, StoreDownloadedDataCallback)) !=
		CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_FOLLOWLOCATION", res);
		goto cleanupLabel;
	}

	// Set the custom parameter of the callback to the memory block.
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_WRITEDATA, (void*)& block)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_WRITEDATA", res);
		goto cleanupLabel;
	}

	// Perform the POST action
	if ((res = curl_easy_perform(curlHandle)) != CURLE_OK) {
		LogCurlError("curl_easy_perform", res);
	}
	else {
		Log_Debug("\n -===- Downloaded content (%zu bytes): -===-\n", block.size);
		Log_Debug("%s\n", block.data);
		// read contents json

		JSON_Value* root_value = NULL;
		root_value = json_parse_string(block.data);
		JSON_Object* root_object;
		if (json_value_get_type(root_value) == JSONObject)
		{
			root_object = json_value_get_object(root_value);
			bool isAnomaly = json_object_get_boolean(root_object, "isAnomaly");

			Log_Debug(isAnomaly ? "true" : "false");

			if (isAnomaly)
				relaystate(rptr, relay1_set);
			else
				relaystate(rptr, relay1_clr);
		}
		if (root_value)json_value_free(root_value);

		//JSON_Value* result = json_parse_string(block.data);
		//JSON_Value* anomaly = json_object_get_value(result, "IsAnomaly");
		//bool isAnomaly = json_object_get_boolean(json_object(result),"IsAnomaly");
		
	}

cleanupLabel:
	// Clean up allocated memory.
	free(block.data);
	free(certificatePath);
	free(subkey);
	// Clean up sample's cURL resources.
	curl_easy_cleanup(curlHandle);
	// Clean up cURL library's resources.
	curl_global_cleanup();
	Log_Debug("\n -===- End of download -===-\n");

exitLabel:
	return;
}

double dround(double val, int dp) {
	int charsNeeded = 1 + snprintf(NULL, 0, "%.*f", dp, val);
	char* buffer = malloc(charsNeeded);
	snprintf(buffer, charsNeeded, "%.*f", dp, val);
	double result = atof(buffer);
	free(buffer);
	return result;
}

/// <summary>
///     The timer event handler.
/// </summary>
static void TimerEventHandler(EventData *eventData)
{
	uint8_t reg;
	lps22hh_reg_t lps22hhReg;

    if (ConsumeTimerFdEvent(webpageDownloadTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

	if (entries->count < 12)
	{
		return;
	}

	char* reportedPropertiesString = NULL;

	JSON_Value* reportedPropertiesRootJson = json_value_init_object();
	if (reportedPropertiesRootJson == NULL) {
		printf("ERROR: could not create the JSON_Value for reporting.\n");
		return;
	}

	JSON_Object* reportedPropertiesJson = json_value_get_object(reportedPropertiesRootJson);
	if (reportedPropertiesJson == NULL) {
		printf("ERROR: could not get the JSON_Object for reporting.\n");
		goto cleanup;
	}

	json_object_set_number(reportedPropertiesJson, "MaxAnomalyRatio", 0.25);
	json_object_set_number(reportedPropertiesJson, "Sensitivity", 95);
	json_object_set_string(reportedPropertiesJson, "Granularity", "minutely");

	JSON_Value* branch = json_value_init_array();
	JSON_Array* leaves = json_value_get_array(branch);

	LIST_FOREACH(current, entries) {
		Entry* entry = (Entry*)current->value;
		JSON_Value* leaf_value = json_value_init_object();
		JSON_Object* leaf_object = json_value_get_object(leaf_value);
		json_object_set_string(leaf_object, "Timestamp", (char*)entry->time);
		json_object_set_number(leaf_object, "Value", (double)entry->data);
		json_array_append_value(leaves, leaf_value);
	};

	char* propertyName = "Series";
	if (JSONSuccess !=
		json_object_set_value(reportedPropertiesJson, propertyName, branch)) {
		printf("ERROR: could not set the property value for Device Twin reporting.\n");
		goto cleanup;
	}

	reportedPropertiesString = json_serialize_to_string(reportedPropertiesRootJson);
	// ("%s", reportedPropertiesString);
	Log_Debug("\n[Info] Sending telemetry: %s\n", reportedPropertiesString);
	//float* value = Ringbuf_Pop_Front(&data_buffer);
	//// construct the telemetry message
	//snprintf(pjsonBuffer, JSONDATA_BUFFER_SIZE, 
	//	"{\"x\":\"%.4lf\"}",
	//	&value);

	// Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
	// AzureIoT_SendMessage(pjsonBuffer);
	/*free(pjsonBuffer);*/

    //PerformWebPageDownload();
	PerformWebhookPOST(reportedPropertiesString);
cleanup:
	if (reportedPropertiesRootJson != NULL) {
		json_value_free(reportedPropertiesRootJson);
	}
}

// event handler data structures. Only the event handler field needs to be populated.
static EventData timerEventData = {.eventHandler = &TimerEventHandler};

struct tm info;
time_t refTime;
size_t minute = 60;

static void ListTimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(listTimerFd) != 0) {
		terminationRequired = true;
		return;
	}


	//time_t t = time(NULL);
	//time_t t = mktime(&info);

	refTime = refTime + minute;
	struct tm tm = *localtime(&refTime);
		printf("now: %d-%02d-%002dT%02d:%02d:%02dZ\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	char* timeBuffer = (char*)malloc(TIMESIZE);
	snprintf(timeBuffer, TIMESIZE, "%d-%02d-%002dT%02d:%02d:%02dZ",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	Entry* entry = malloc(sizeof(Entry));
	double value = (double)tracked_value;
	entry->data = value;
	entry->time = timeBuffer;

	List_push(entries, entry);

	if (entries->count > 200) {
		List_shift(entries);
	}
	tracked_value = 0;
}

static EventData listTimerEventData = { .eventHandler = &ListTimerEventHandler };

static void SensorTimerEventHandler(EventData* eventData)
{
	uint8_t reg;
	lps22hh_reg_t lps22hhReg;

	if (ConsumeTimerFdEvent(sensorTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	// Read the sensors on the lsm6dso device

	//Read output only if new xl value is available
	lsm6dso_xl_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
		// Read acceleration field data
		memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

		acceleration_mg[0] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[0]);
		acceleration_mg[1] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[1]);
		acceleration_mg[2] = lsm6dso_from_fs4_to_mg(data_raw_acceleration.i16bit[2]);

		/*if (abs(acceleration_mg[1]) > tracked_value)
		{
			tracked_value = abs(acceleration_mg[1]);
		}*/
		tracked_value = acceleration_mg[1];
		Log_Debug("\nLSM6DSO: Acceleration [mg]  : %.4lf, %.4lf, %.4lf\n",
			acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
	}

	lsm6dso_gy_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
		// Read angular rate field data
		memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);

		// Before we store the mdps values subtract the calibration data we captured at startup.
		angular_rate_dps[0] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0] - raw_angular_rate_calibration.i16bit[0])) / 1000.0;
		angular_rate_dps[1] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1] - raw_angular_rate_calibration.i16bit[1])) / 1000.0;
		angular_rate_dps[2] = (lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2] - raw_angular_rate_calibration.i16bit[2])) / 1000.0;

		Log_Debug("LSM6DSO: Angular rate [dps] : %4.2f, %4.2f, %4.2f\r\n",
			angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2]);

	}

	lsm6dso_temp_flag_data_ready_get(&dev_ctx, &reg);
	if (reg)
	{
		// Read temperature data
		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lsm6dso_temperature_raw_get(&dev_ctx, data_raw_temperature.u8bit);
		lsm6dsoTemperature_degC = lsm6dso_from_lsb_to_celsius(data_raw_temperature.i16bit);

		/*time_t t = time(NULL);
		struct tm tm = *localtime(&t);
		// printf("now: %d-%02d-%002dT%02d:%02d:%02dZ\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		char* timeBuffer = (char*)malloc(TIMESIZE);
		snprintf(timeBuffer, TIMESIZE, "%d-%02d-%002dT%02d:%02d:%02dZ",
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		Entry* entry = malloc(sizeof(Entry));
		double value = (double)((int)(lsm6dsoTemperature_degC * 100 + .5)) / 100;
		entry->data = value;
		entry->time = timeBuffer;

		List_push(entries, entry);*/
		// status = Ringbuf_Put(&data_buffer, &lsm6dsoTemperature_degC);
		// double value = (double)((int)(lsm6dsoTemperature_degC * 100 + .5)) / 100;
		// double value = dround(lsm6dsoTemperature_degC, 2);		
		/*tracked_value = lsm6dsoTemperature_degC;*/
		Log_Debug("LSM6DSO: Temperature  [degC]: %.2f\r\n", tracked_value);
	}

	// Read the sensors on the lsm6dso device

	lps22hh_read_reg(&pressure_ctx, LPS22HH_STATUS, (uint8_t*)& lps22hhReg, 1);

	//Read output only if new value is available

	if ((lps22hhReg.status.p_da == 1) && (lps22hhReg.status.t_da == 1))
	{
		memset(data_raw_pressure.u8bit, 0x00, sizeof(int32_t));
		lps22hh_pressure_raw_get(&pressure_ctx, data_raw_pressure.u8bit);

		pressure_hPa = lps22hh_from_lsb_to_hpa(data_raw_pressure.i32bit);

		memset(data_raw_temperature.u8bit, 0x00, sizeof(int16_t));
		lps22hh_temperature_raw_get(&pressure_ctx, data_raw_temperature.u8bit);
		lps22hhTemperature_degC = lps22hh_from_lsb_to_celsius(data_raw_temperature.i16bit);

		Log_Debug("LPS22HH: Pressure     [hPa] : %.2f\r\n", pressure_hPa);
		Log_Debug("LPS22HH: Temperature  [degC]: %.2f\r\n", lps22hhTemperature_degC);
	}


}

static EventData sensorTimerEventData = { .eventHandler = &SensorTimerEventHandler };

/// <summary>
///     Set up SIGTERM termination handler and event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

	if (initI2c() == -1) {
		return -1;
	}

	struct timespec sensorTimerSpan = { 1, 0 };
	sensorTimerFd =
		CreateTimerFdAndAddToEpoll(epollFd, &sensorTimerSpan, &sensorTimerEventData, EPOLLIN);
	if (sensorTimerFd < 0) {
		return -1;
	}

    // Issue an HTTPS request at the specified period.
    struct timespec listTimeSpan = { 1, 0};
	listTimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &listTimeSpan, &listTimerEventData, EPOLLIN);
    if (listTimerFd < 0) {
        return -1;
    }

	// Issue an HTTPS request at the specified period.
	struct timespec webpagedownloadSpan = { 1, 0 }; // 2 seconds
	webpageDownloadTimerFd =
		CreateTimerFdAndAddToEpoll(epollFd, &webpagedownloadSpan, &timerEventData, EPOLLIN);
	if (webpageDownloadTimerFd < 0) {
		return -1;
	}
	
    return 0;
}

/// <summary>
///     Clean up the resources previously allocated.
/// </summary>
static void CloseHandlers(void)
{
    // Close the timer and epoll file descriptors.
    CloseFdAndPrintError(webpageDownloadTimerFd, "WebpageDownloadTimer");
    CloseFdAndPrintError(epollFd, "Epoll");
}

#define MIKROE_PWM  MT3620_GPIO1   //click#1=GPIO0;  click#2=GPIO1
#define MIKROE_CS   MT3620_GPIO35  //click#1=GPIO34; click#2=GPIO35

static int r1PinFd;  //relay #1
static GPIO_Value_Type relay1Pin;
static int r2PinFd;  //relay #2
static GPIO_Value_Type relay2Pin;



void init(void)
{
	r1PinFd = GPIO_OpenAsOutput(MIKROE_PWM, relay1Pin, GPIO_Value_Low);
	r2PinFd = GPIO_OpenAsOutput(MIKROE_CS, relay2Pin, GPIO_Value_Low);
}

void    state(RELAY* ptr)
{
	if (ptr->relay1_status == 1)
		GPIO_SetValue(r1PinFd, GPIO_Value_High);
	else
		GPIO_SetValue(r1PinFd, GPIO_Value_Low);

	if (ptr->relay2_status == 1)
		GPIO_SetValue(r2PinFd, GPIO_Value_High);
	else
		GPIO_SetValue(r2PinFd, GPIO_Value_Low);
}


/// <summary>
///     Main entry point for this sample.
/// </summary>
int main(int argc, char *argv[])
{
    //Log_Debug("cURL easy interface based application starting.\n");
    //Log_Debug("This sample periodically attempts to download a webpage, using curl's 'easy' API.");
	sleep(30); //waiting for wifi

	//init();
	rptr = open_relay(state, init);
	sleep(1);

	bool isNetworkingReady = false;
	if ((Networking_IsNetworkingReady(&isNetworkingReady) < 0) || !isNetworkingReady) {
		Log_Debug("\nNot doing download because network is not up.\n");

		relaystate(rptr, relay1_set);
		sleep(1);
	}


	relaystate(rptr, relay1_set);
	sleep(1);
	relaystate(rptr, relay1_clr);
	sleep(1);

	info.tm_year = 2019 - 1900;
	info.tm_mon = 0;
	info.tm_mday = 1;
	info.tm_hour = 0;
	info.tm_min = 0;
	info.tm_sec = 0;
	info.tm_isdst = -1;
	refTime = mktime(&info);
	struct tm tm = *localtime(&refTime);
	Log_Debug("now: %d-%02d-%002dT%02d:%02d:%02dZ\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	
	refTime = refTime + (minute);
	tm = *localtime(&refTime);
	Log_Debug("now: %d-%02d-%002dT%02d:%02d:%02dZ\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	// Ringbuf_Init(&data_buffer, &data_store, element_size, element_count);
	entries = List_create();

	
    if (InitHandlers() != 0) {
        terminationRequired = true;
    } else {
        // Download the web page immediately.
        PerformWebPageDownload();
		//char* reportedPropertiesString = "{\"MaxAnomalyRatio\":0.25,\"Sensitivity\":95,\"Granularity\":\"minutely\",\"Series\":[{\"Timestamp\":\"2019-09-23T03:50:10Z\",\"Value\":32.200000000000003},{\"Timestamp\":\"2019-09-23T03:51:10Z\",\"Value\":32.619999999999997},{\"Timestamp\":\"2019-09-23T03:52:10Z\",\"Value\":32.899999999999999},{\"Timestamp\":\"2019-09-23T03:53:10Z\",\"Value\":33.18},{\"Timestamp\":\"2019-09-23T03:54:10Z\",\"Value\":33.270000000000003},{\"Timestamp\":\"2019-09-23T03:55:10Z\",\"Value\":33.340000000000003},{\"Timestamp\":\"2019-09-23T03:56:10Z\",\"Value\":33.5},{\"Timestamp\":\"2019-09-23T03:57:10Z\",\"Value\":33.469999999999999},{\"Timestamp\":\"2019-09-23T03:58:10Z\",\"Value\":33.619999999999997},{\"Timestamp\":\"2019-09-23T03:59:10Z\",\"Value\":33.700000000000003},{\"Timestamp\":\"2019-09-23T04:00:10Z\",\"Value\":33.740000000000002},{\"Timestamp\":\"2019-09-23T04:01:10Z\",\"Value\":133.939999999999998}]}";
		//PerformWebhookPOST(reportedPropertiesString);
    }

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }


		//nanosleep(&LoopInterval, NULL);
    }

    CloseHandlers();

	close_relay(rptr);
	GPIO_SetValue(r1PinFd, GPIO_Value_Low);
	GPIO_SetValue(r2PinFd, GPIO_Value_Low);
	Log_Debug("relay2 OFF, relay1 oFF\nDONE...\n");

    Log_Debug("Application exiting.\n");
    return 0;
}
