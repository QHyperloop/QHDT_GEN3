#include <stdio.h>
#include <curl/curl.h>

int main(void) {
    CURL *curl;
    CURLcode res;

    // Initialize the curl session
    curl = curl_easy_init();
    if (curl) {
        // Set the URL
        curl_easy_setopt(curl, CURLOPT_URL, "http://localhost:3000/sensor-data");

        // Set the data to be sent (replace with your actual sensor data)
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "{\"sensorValue\": 123.45}");

        // Perform the HTTP POST request
        res = curl_easy_perform(curl);

        // Cleanup
        curl_easy_cleanup(curl);
    }

    return 0;
}