// #include "minunit.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include "lcthw/list.h"
#include "lcthw/list_algos.h"
#include <time.h>
#include "parson.h"
#include "relay.h"
#include "applibs_versions.h"
#include <applibs/log.h>

#include "mt3620_rdb.h"

#include <unistd.h>

char* values[] = { "XXXX", "1234", "abcd", "xjvef", "NDSS" };

#define NUM_VALUES 5

void print_words(List* words, char* msg)
{
	printf("%s", msg);
	LIST_FOREACH(curr, words) {
		printf("%s ", (char*)curr->value);
	}

	printf("\n");
}

List* create_words()
{
	int i = 0;

	List* words = List_create();

	for (i = 0; i < NUM_VALUES; i++)
	{
		List_push(words, values[i]);
	}

	return words;
}

typedef struct Entry {
	double data; /* block of memory or array of data */
	char* time;
} Entry;
#define TIMESIZE 128


void list()
{
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	printf("now: %d-%02d-%002dT%02d:%02d:%02dZ\n", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	char* timeBuffer = (char*)malloc(TIMESIZE);
	snprintf(timeBuffer, TIMESIZE, "%d-%02d-%002dT%02d:%02d:%02dZ",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	Entry* entry = malloc(sizeof(Entry));
	double var = 100.12355;

	double value = (double)((int)(var * 100 + .5)) / 100;

	entry->data = value;
	entry->time = timeBuffer;

	List* entries = List_create();
	List_push(entries, entry);
	List_push(entries, entry);
	List_push(entries, entry);

	LIST_FOREACH(curr, entries) {
		Entry* entry = (Entry*)curr->value;
		float val = entry->data;
		printf("Entry %s %.2d\n", (char*)entry->time, val);
	}

	printf("\n");

	List* words = create_words();

	print_words(words, "Contents");


	//convert to json
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

	json_object_set_number(reportedPropertiesJson, "maxAnomalyRatio", 0.25);
	json_object_set_number(reportedPropertiesJson, "sensitivity", 95);
	json_object_set_string(reportedPropertiesJson, "granularity", "minutely");

	JSON_Value* branch = json_value_init_array();
	JSON_Array* leaves = json_value_get_array(branch);

	LIST_FOREACH(current, entries) {
		Entry* entry = (Entry*)current->value;
		JSON_Value* leaf_value = json_value_init_object();
		JSON_Object* leaf_object = json_value_get_object(leaf_value);
		json_object_set_string(leaf_object, "timestamp", (char*)entry->time);
		json_object_set_number(leaf_object, "value", (double)entry->data);
		json_array_append_value(leaves, leaf_value);
	};


	char* propertyName = "Series";
	if (JSONSuccess !=
		json_object_set_value(reportedPropertiesJson, propertyName, branch)) {
		printf("ERROR: could not set the property value for Device Twin reporting.\n");
		goto cleanup;
	}

	reportedPropertiesString = json_serialize_to_string(reportedPropertiesRootJson);
	printf("%s", reportedPropertiesString);
cleanup:
	if (reportedPropertiesRootJson != NULL) {
		json_value_free(reportedPropertiesRootJson);
	}

	/*if (leaf_object != NULL) {
		json_object_clear(leaf_object);
	}

	if (leaf_value != NULL) {
		json_value_free(leaf_value);
	}
	if (leaves != NULL) {
		json_array_clear(leaves);
	}

	if (branch != NULL) {
		json_value_free(branch);
	};*/



	if (reportedPropertiesString != NULL) {
		json_free_serialized_string(reportedPropertiesString);
	}
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

int main(void)
{

	int    i, run_time = 30;  //default to 30 second run time
	RELAY* rptr;

	rptr = open_relay(state, init);
	sleep(1);

	relaystate(rptr, relay1_set );
	sleep(1);
	relaystate(rptr, relay1_clr);
	sleep(1);
	relaystate(rptr, relay1_set);
	sleep(1);
	/*i = 0;
	while (i++ < run_time) {
		relaystate(rptr, (i & 1) ? relay1_set : relay1_clr);
		relaystate(rptr, (i & 2) ? relay2_set : relay2_clr);
		Log_Debug("(%d) relay2 %s, relay1 %s\n", i,
			relaystate(rptr, relay2_rd) ? "ON" : "OFF",
			relaystate(rptr, relay1_rd) ? "ON" : "OFF");
		sleep(1);
	}*/

	close_relay(rptr);
	GPIO_SetValue(r1PinFd, GPIO_Value_Low);
	GPIO_SetValue(r2PinFd, GPIO_Value_Low);
	Log_Debug("relay2 OFF, relay1 oFF\nDONE...\n");
	exit(EXIT_SUCCESS);

	// list();
}

