/*
 * Copyright (c) 2020 Arm Limited
 * SPDX-License-Identifier: Apache-2.0
 */
// #include "cellular_app.h"

#include "mbed.h"
#include "rtos/ThisThread.h"
#include "NTPClient.h"

#include "cellular_service_power.h"

#include "certs.h"
#include "iothub.h"
#include "iothub_client_options.h"
#include "iothub_device_client.h"
#include "iothub_message.h"
#include "azure_c_shared_utility/shared_util_options.h"
#include "azure_c_shared_utility/threadapi.h"
#include "azure_c_shared_utility/tickcounter.h"
#include "azure_c_shared_utility/xlogging.h"

#include "iothubtransportmqtt.h"
#include "azure_cloud_credentials.h"

#include "../LSM6DSL/LSM6DSL_acc_gyro_driver.h"
#include "../LSM6DSL/LSM6DSLSensor.h"
#include "ei_run_classifier.h"
#include "model-parameters/model_metadata.h"
#include <cstring>

#include 

// Set the sampling frequency in Hz
static int16_t sampling_freq = 101;
static int64_t time_between_samples_us = (1000000 / (sampling_freq - 1));

static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// Blinking rate in milliseconds
#define BLINKING_RATE    500ms
#define BUSY_LOOP_DURATON 9ms

// Serial 
/**
 * This example sends and receives messages to and from Azure IoT Hub.
 * The API usages are based on Azure SDK's official iothub_convenience_sample.
 */

 // Check if this checks out for B-L4S5I board
static DevI2C devI2C(PB_11, PB_10);
// Acc and Gyro Class Object
static LSM6DSLSensor acc_gyro(&devI2C, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW);


// Global symbol referenced by the Azure SDK's port for Mbed OS, via "extern"
NetworkInterface *_defaultSystemNetwork;

static bool message_received = false;

static void on_connection_status(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void* user_context)
{
    if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED) {
        LogInfo("Connected to IoT Hub");
    } else {
        LogError("Connection failed, reason: %s", MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONNECTION_STATUS_REASON, reason));
    }
}

static IOTHUBMESSAGE_DISPOSITION_RESULT on_message_received(IOTHUB_MESSAGE_HANDLE message, void* user_context)
{
    LogInfo("Message received from IoT Hub");

    const unsigned char *data_ptr;
    size_t len;
    if (IoTHubMessage_GetByteArray(message, &data_ptr, &len) != IOTHUB_MESSAGE_OK) {
        LogError("Failed to extract message data, please try again on IoT Hub");
        return IOTHUBMESSAGE_ABANDONED;
    }

    message_received = true;
    LogInfo("Message body: %.*s", len, data_ptr);
    return IOTHUBMESSAGE_ACCEPTED;
}

static void on_message_sent(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
    if (result == IOTHUB_CLIENT_CONFIRMATION_OK) {
        LogInfo("Message sent successfully");
    } else {
        LogInfo("Failed to send message, error: %s",
            MU_ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
    }
}

int count_for_loop = 0;
IOTHUB_DEVICE_CLIENT_HANDLE client_handle;
IOTHUB_CLIENT_RESULT res;
tickcounter_ms_t interval = 100;
void demo() {
    // Machine Learning Setup
    void *init;

    int32_t acc_val_buf[3] = {0};
    int32_t gyro_val_buf[3] = {0};
    // init initializes the component
    acc_gyro.init(init);
    // enables the accelero
    acc_gyro.enable_x();
    // enable gyro
    acc_gyro.enable_g();

    // int y = bar(20);
    Timer t;
    t.start();

    bool trace_on = MBED_CONF_APP_IOTHUB_CLIENT_TRACE;
    // tickcounter_ms_t interval = 100;
    // IOTHUB_CLIENT_RESULT res;

    LogInfo("Initializing IoT Hub client");
    IoTHub_Init();

    client_handle = IoTHubDeviceClient_CreateFromConnectionString(
        azure_cloud::credentials::iothub_connection_string,
        MQTT_Protocol
    );
    if (client_handle == nullptr) {
        LogError("Failed to create IoT Hub client handle");
        goto cleanup;
    }

    // Enable SDK tracing
    res = IoTHubDeviceClient_SetOption(client_handle, OPTION_LOG_TRACE, &trace_on);
    if (res != IOTHUB_CLIENT_OK) {
        LogError("Failed to enable IoT Hub client tracing, error: %d", res);
        goto cleanup;
    }

    // Enable static CA Certificates defined in the SDK
    res = IoTHubDeviceClient_SetOption(client_handle, OPTION_TRUSTED_CERT, certificates);
    if (res != IOTHUB_CLIENT_OK) {
        LogError("Failed to set trusted certificates, error: %d", res);
        goto cleanup;
    }

    // Process communication every 100ms
    res = IoTHubDeviceClient_SetOption(client_handle, OPTION_DO_WORK_FREQUENCY_IN_MS, &interval);
    if (res != IOTHUB_CLIENT_OK) {
        LogError("Failed to set communication process frequency, error: %d", res);
        goto cleanup;
    }

    // set incoming message callback
    res = IoTHubDeviceClient_SetMessageCallback(client_handle, on_message_received, nullptr);
    if (res != IOTHUB_CLIENT_OK) {
        LogError("Failed to set message callback, error: %d", res);
        goto cleanup;
    }

    // Set connection/disconnection callback
    res = IoTHubDeviceClient_SetConnectionStatusCallback(client_handle, on_connection_status, nullptr);
    if (res != IOTHUB_CLIENT_OK) {
        LogError("Failed to set connection status callback, error: %d", res);
        goto cleanup;
    }

    while(true) {
        // Do machine learning
        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
            int64_t next_tick = t.read_us() + time_between_samples_us;
            // printf("gyro enable val: %d\n", check_ena_g);
            acc_gyro.get_x_axes(acc_val_buf);
            acc_gyro.get_g_axes(gyro_val_buf);

            features[ix + 0] = static_cast<float>(acc_val_buf[0]) / 100.0f;
            features[ix + 1] = static_cast<float>(acc_val_buf[1]) / 100.0f;
            features[ix + 2] = static_cast<float>(acc_val_buf[2]) / 100.0f;
            features[ix + 3] = static_cast<float>(gyro_val_buf[0]) / 1000.0f;
            features[ix + 4] = static_cast<float>(gyro_val_buf[1]) / 1000.0f;
            features[ix + 5] = static_cast<float>(gyro_val_buf[2]) / 1000.0f;
            while (t.read_us() < next_tick){
                // busy loop
            }
        }

        ei_impulse_result_t result = {0};

        signal_t signal;

        numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

        // run classifier
        EI_IMPULSE_ERROR ei_res = run_classifier(&signal, &result, false);
        ei_printf("run_classifier returned: %d\n", ei_res);
        // if (res != 0) return 1;

        // print predictions
        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

        int largest_index_val = 0;
        int largest_val = 0.f;
        // print the predictions
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (static_cast<int>(result.classification[ix].value*100) > largest_val)
            {
                largest_val = static_cast<int>(result.classification[ix].value*100);
                largest_index_val = ix;
            }
            ei_printf("%s:\t%.5f\n", result.classification[ix].label, result.classification[ix].value);
        }
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf("anomaly:\t%.3f\n", result.anomaly);
        #endif

        if (count_for_loop > 1000){
            count_for_loop = 0;
        }


        IOTHUB_MESSAGE_HANDLE message_handle;
        char message[80];
        int DeviceID = 1583;               // Device Identifier
        char NN_State[20] = "LOL";       // User state (Stoop, Walk, Stand, Squat, Unused)
        int Duration = 400;             // Duration of state [ms] 

        switch (largest_index_val) {
            case 0: strcpy(NN_State, "Squat");
                    break;
            case 1: strcpy(NN_State, "Stand");
                    break;
            case 2: strcpy(NN_State, "Stoop");
                    break;
            case 3: strcpy(NN_State, "Walk");
                    break;
            default: strcpy(NN_State, "Anomaly");
                    break;
        }

        sprintf(message, "{\"DeviceID\":%d,\"State\":\"%s\",\"Duration\":%d}", DeviceID, NN_State, Duration);



        // sprintf(message, "%d messages left to send, or until we receive a reply", count_for_loop++);
        // sprintf(message, "ANOMALY IS: %.3f", result.anomaly);
        LogInfo("Sending: \"%s\"", message);

        message_handle = IoTHubMessage_CreateFromString(message);
        if (message_handle == nullptr) {
            LogError("Failed to create message");
            goto cleanup;
        }

        res = IoTHubDeviceClient_SendEventAsync(client_handle, message_handle, on_message_sent, nullptr);
        IoTHubMessage_Destroy(message_handle); // message already copied into the SDK

        if (res != IOTHUB_CLIENT_OK) {
            LogError("Failed to send message event, error: %d", res);
            goto cleanup;
        }


        ThisThread::sleep_for(5s); 
    }

cleanup:
    IoTHubDeviceClient_Destroy(client_handle);
    IoTHub_Deinit();
}

int main() {
    LogInfo("Connecting to the network");

    _defaultSystemNetwork = NetworkInterface::get_default_instance();
    if (_defaultSystemNetwork == nullptr) {
        LogError("No network interface found");
        return -1;
    }

    int ret = _defaultSystemNetwork->connect();
    if (ret != 0) {
        LogError("Connection error: %d", ret);
        return -1;
    }
    LogInfo("Connection success, MAC: %s", _defaultSystemNetwork->get_mac_address());

    LogInfo("Getting time from the NTP server");

    NTPClient ntp(_defaultSystemNetwork);
    ntp.set_server("2.pool.ntp.org", 123);
    time_t timestamp = ntp.get_timestamp();
    if (timestamp < 0) {
        LogError("Failed to get the current time, error: %ld", timestamp);
        return -1;
    }
    LogInfo("Time: %s", ctime(&timestamp));
    set_time(timestamp);



    LogInfo("Starting the Demo");
    demo();
    LogInfo("The demo has ended");



    return 0;
}
