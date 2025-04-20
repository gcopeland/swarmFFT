#pragma once

#ifndef SWARMFFT_H_
#define SWARMFFT_H_

/*
 * Smarm Audio FFT detection logic
 *
 * 50Hz - 1100Hz = 1050Hz spectrum
 *
 * Normal Detection Range:
 * 150Hz - 250Hz w/ Power peak 200Hz
 * 300Hz - 500Hz w/ Power peak 385Hz
 * 550Hz - 585Hz w/ Power peak 575Hz
 *
 * Swarm Detection Range:
 * 50Hz - 250Hz w/ Power peak 175Hz
 * 925Hz - 975Hz w/ Power peak 950Hz
 *
 * These are adjustable by config file.
 *
 */

#include <stdint.h>

#include <string>

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/mqtt/custom_mqtt_device.h"

#include <ArduinoJson.h>

#include <AudioTools.h>
#include <AudioLibs/AudioRealFFT.h>


/*
** These must be declared in the including configuration
** file. This allows for different nodes to have different
** pins.
 */
extern const uint8_t PIN_MIC_WS;
extern const uint8_t PIN_MIC_CLOCK;
extern const uint8_t PIN_MIC_DATA;
extern const uint8_t PIN_MIC_DATA_IN;

namespace esphome {
namespace swarm_audio {
    // Our logging component name
    static const char *TAG = "SwarmFFT";

    // Configs
    static const uint16_t CHANNELS = 2;
    static const uint16_t BITS_PER_SAMPLE = 32;
    static const uint16_t SAMPLE_LENGTH = 512;
    static const uint16_t MAX_FREQUENCY_HZ = 2000;
    static const uint16_t SAMPLES_PER_SECOND = MAX_FREQUENCY_HZ * 2;
    static const uint16_t FFT_BINS = SAMPLE_LENGTH/2;

    // Forward declaration FFT handler
    static void fftCallback(AudioFFTBase &fft);

    class SwarmFFT : public esphome::Component,
                     public esphome::mqtt::CustomMQTTDevice {
        public:
            // Data pin
            const uint8_t wsPin_;
            const uint8_t clockPin_;
            const uint8_t dataPin_;

            // Quick reference strings - constructed
            std::string name_;
            std::string prefix_;
            std::string state_topic_;
            std::string command_topic_;
            std::string discovery_topic_;

            explicit SwarmFFT() {};

            explicit SwarmFFT(uint8_t ws, uint8_t clock, uint8_t data,
                              uint32_t pollingInterval,
                              const std::string &prefix, const std::string &name) :
                copier_(fft_, i2s_),
                prefix_(prefix),
                name_(name),
                wsPin_(ws), clockPin_(clock), dataPin_(data),
                lastCopy_(0),
                incompleteAudio_(true),
                update_interval_(pollingInterval) {

                // Setup our MQTT topics
                state_topic_ = prefix + "/" + name_ + "/microphone/" + "state" ;
                command_topic_ = prefix + "/" + name_ + "/microphone/" + "command";

                auto const &discoveryInfo = esphome::mqtt::global_mqtt_client->get_discovery_info();
                discovery_topic_ = discoveryInfo.prefix + "/sensor/" + name_ + "/" + name_ + "_microphone/config";

                // Log our MQTT topics
                ESP_LOGI(TAG, "state: %s", state_topic_.c_str());
                ESP_LOGI(TAG, "command: %s", command_topic_.c_str());
                ESP_LOGI(TAG, "discovery: %s", discovery_topic_.c_str());
                ESP_LOGD(TAG, "Instantiation complete");
            }


            void setup() override {
                // Configuring audio toolkit logging
                ESP_LOGD(TAG, "setup() start");
                AudioLogger::instance().begin(Serial, AudioLogger::Warning);

                // Subscribe to our MQTT command topic
                subscribe_json(command_topic_, &SwarmFFT::on_json_message);

                // Setup reading from our I2S mic
                auto cfg = i2s_.defaultConfig(RX_MODE);
                cfg.i2s_format = I2S_STD_FORMAT;
                cfg.bits_per_sample = BITS_PER_SAMPLE;
                cfg.sample_rate = SAMPLES_PER_SECOND;
                cfg.channels = CHANNELS;
                cfg.is_master = true;
                cfg.use_apll = true;
                cfg.port_no = 0;
                cfg.pin_ws = wsPin_;
                cfg.pin_bck = clockPin_;
                cfg.pin_data = dataPin_;
                i2s_.begin(cfg);

                // Setup our FFT w/callback and basic hamming windowing
                fftCfg_ = fft_.defaultConfig();
                fftCfg_.bits_per_sample = cfg.bits_per_sample;
                fftCfg_.sample_rate = cfg.sample_rate;
                fftCfg_.length = SAMPLE_LENGTH;
                fftCfg_.channels = cfg.channels;
                fftCfg_.window_function = new BufferedWindow(new Hamming());
                fftCfg_.callback = fftCallback;
                fft_.begin(fftCfg_);

                ESP_LOGD(TAG, "setup() complete");
            }


            // Announce ourselves to home assistant
            void doDiscovery() {
                if((!discoveryComplete_) && is_connected()) {
                    discoveryComplete_ = publish_json(discovery_topic_,
                                                      [=](JsonObject root) {
                                                          esphome::mqtt::SendDiscoveryConfig config;
                                                          config.state_topic = true;
                                                          config.command_topic = true;

                                                          root["name"] = name_ + "_swarm_microphone_fft";
                                                          root["stat_cla"] = "measurement";
                                                          root["suggested_area"] = "hives";
                                                          root[esphome::mqtt::MQTT_STATE_TOPIC] = state_topic_;
                                                          root[esphome::mqtt::MQTT_COMMAND_TOPIC] = command_topic_;
                                                          root[esphome::mqtt::MQTT_JSON_ATTRIBUTES_TOPIC] = state_topic_;
                                                          root[esphome::mqtt::MQTT_UNIQUE_ID] = this->name_ + "_microphone_" + get_mac_address();
                                                          root[esphome::mqtt::MQTT_ICON] = "mdi:microphone-plus";
                                                          root[esphome::mqtt::MQTT_JSON_ATTRIBUTES_TEMPLATE] = "{\"bin\":\"{{value}}\"}";

                                                          auto dev = root.createNestedObject("dev");
                                                          dev[esphome::mqtt::MQTT_DEVICE_NAME] = name_;
                                                          dev[esphome::mqtt::MQTT_DEVICE_IDENTIFIERS] = "ESP_MICROPHONE_" + get_mac_address();
                                                          //dev[esphome::mqtt::MQTT_DEVICE_SW_VERSION] = esphome::ESPHOME_VERSION;
                                                          dev[esphome::mqtt::MQTT_DEVICE_MANUFACTURER] = "gtcopeland";
                                                  }, 2, true);
                }
            }

            void on_json_message(JsonObject root) {
                ESP_LOGI(TAG, "MQTT on_json_message");
                if(!root.containsKey("root")) {
                    ESP_LOGE(TAG, "root node not provided");
                }
            }

            void on_shutdown() override {
                ESP_LOGI(TAG, "CHIPEN pin not exposed. Sleep mode not available.");
            }


            void loop() override {
                // Copy the data to our FFT which will trigger our FFT callback
                // for processing. This works by waiting for the update interval
                // to expire OR continuing copying until enough audio data is
                // available for FFT processing. Once enough data is copied, it
                // begins to wait for the update interval again.
                doDiscovery();
                auto now = millis();
                if(is_connected() && incompleteAudio_ ||
                   (now - lastCopy_ >= update_interval_)) {
                    incompleteAudio_ = true;
                    ESP_LOGD(TAG, "copier");
                    copier_.copy();
                    lastCopy_ = now;
                }

                // Process FFT data if it's available
                if(haveFFTResult()) {
                    incompleteAudio_ = false;
                    processFFTResult();
                }
            }


            bool haveFFTResult() const {
                return haveFFTResult_;
            }


            void reportFFTResult(AudioFFTBase &fft) {
                if(haveFFTResult_) {
                    ESP_LOGE(TAG, "FFT delivered with pending FFT.");
                } else {
                    fft.resultArray(fftResult_);
                    haveFFTResult_ = true;
                }
            }


            // Tell it we are only data and can start later
            float get_setup_priority() const override {
                return esphome::setup_priority::IO;
            }

        private:
            I2SStream i2s_;
            AudioRealFFT fft_;
            AudioFFTConfig fftCfg_;
            StreamCopy copier_;
            uint32_t lastCopy_;
            bool incompleteAudio_;
            AudioFFTResult fftResult_[FFT_BINS] = {0};
            uint32_t update_interval_;
            volatile bool haveFFTResult_;
            static const uint16_t MQTT_FFT_STRIPES = 16;
            bool discoveryComplete_ = false ;


            void processFFTResult() {
                if(is_connected()) {
                    // Only try discovery if we're connected
                    doDiscovery();
                    yield();

                    // Break our FFT data in stripes - MQTT_FFT_STRIPES count
                    // ESP_LOGD(TAG, "STRIPES: %d", MQTT_FFT_STRIPES);
                    for(auto stripe=0; stripe < MQTT_FFT_STRIPES; stripe++) {
                        auto pubResult = false;
                        while( pubResult == false ) {
                            pubResult = 1 == publish_json(state_topic_,
                                                          [=](JsonObject root) {
                                                              auto stripeLength = FFT_BINS/MQTT_FFT_STRIPES;
                                                              root["node"] = name_;
                                                              root["stripe"] = stripe;
                                                              auto dataDoc = root.createNestedArray("data");
                                                              for(auto index=0; index < stripeLength; index++) {
                                                                  auto bin = (stripe * stripeLength) + index;
                                                                  auto binDoc = dataDoc.createNestedObject();
                                                                  binDoc["bin"] = bin;
                                                                  binDoc["frequency"] = fftResult_[bin].frequency;
                                                                  binDoc["magnitude"] = fftResult_[bin].magnitude;
                                                                  yield();
                                                              }
                                                          }, 1, false);
                            yield();
                            if(pubResult == false) {
                                // Allow us to bail if we lose connection
                                pubResult = !is_connected();
                            }
                        };
                    }
                    ESP_LOGD(TAG, "Published %d stripes of data to: %s", MQTT_FFT_STRIPES, state_topic_.c_str());
                }
                incompleteAudio_ = false;
                haveFFTResult_ = false;
            }

    };

    /*
    ** =======================================================
    ** Declare functions to handle our global reference to our
    ** swarm singleton and process our FFT results.
     */
    // Global access functions and state
    static SwarmFFT *globalSwarmFFT_;
    SwarmFFT *setGlobalSwarmFFT(SwarmFFT *swarmFFT) {
        globalSwarmFFT_ = swarmFFT;
        return globalSwarmFFT_;
    }


    SwarmFFT *getGlobalSwarmFFT() {
        return globalSwarmFFT_;
    }

    /*
    ** The Audio FFT handler
    */
    static void fftCallback(AudioFFTBase &fft) {
        SwarmFFT *swarmFFT = getGlobalSwarmFFT();
        if(swarmFFT != NULL) {
            swarmFFT -> reportFFTResult(fft);
            ESP_LOGD(TAG, "FFT size: %d", fft.size());
            yield();
        } else {
            ESP_LOGE(TAG, "SwarmFFT instance is missing and NOT globally available.");
        }
    }

} // swarm_audio namespace
} // esphome namespace

#endif // SWARMFFT_H_
