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

#include "swarmfft.hpp"

/*
** These must be declared in the including configuration
** file. This allows for different nodes to have different
** pins.
 */

namespace esphome {
namespace swarm_fft_audio {
    SwarmFFT::SwarmFFT() :
        copier_(fft_, i2s_),
        wsPin_(0),
        clockPin_(0),
        dataPin_(0),
        lastCopy_(0),
        incompleteAudio_(true),
        update_interval_(0xFFFF) {
        ESP_LOGD(TAG, "SwarmFFT() instantiated w/invalid defaults");
        ESP_LOGI(TAG, "ESPHhome prevents RAII");
    }

    void SwarmFFT::setup() {
        // Configuring audio toolkit logging
        ESP_LOGD(TAG, "SwarmFFT::setup() start");
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

        ESP_LOGD(TAG, "SwarmFFT::setup() complete");
        dump_config();
    }


    // Announce ourselves to home assistant
    void SwarmFFT::doDiscovery() {
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
                                                  dev[esphome::mqtt::MQTT_DEVICE_SW_VERSION] = ESPHOME_VERSION;
                                                  dev[esphome::mqtt::MQTT_DEVICE_MANUFACTURER] = "gtcopeland";
                                              }, 2, true);
        }
    }

    void SwarmFFT::on_json_message(JsonObject root) {
        const char *TAG = this -> get_component_source();
        ESP_LOGI(TAG, "MQTT on_json_message");
        if(!root.containsKey("root")) {
            ESP_LOGE(TAG, "root node not provided");
        }
    }

    void SwarmFFT::on_shutdown() {
        ESP_LOGI(TAG, "CHIPEN pin not exposed. Sleep mode not available.");
    }


    void SwarmFFT::loop() {
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


    void SwarmFFT::reportFFTResult(AudioFFTBase &fft) {
        if(haveFFTResult_) {
            ESP_LOGE(TAG, "FFT delivered with pending FFT.");
        } else {
            fft.resultArray(fftResult_);
            haveFFTResult_ = true;
        }
    }


    void SwarmFFT::processFFTResult() {
        if(is_connected()) {
            // Only try discovery if we're connected
            doDiscovery();
            yield();

            // Break our FFT data in stripes - MQTT_FFT_STRIPES count
            ESP_LOGD(TAG, "STRIPES: %d", MQTT_FFT_STRIPES);
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


    void SwarmFFT::setMqttTopicPrefix(std::string prefix) {
        // Setup our MQTT topics
        state_topic_ = prefix + "/" + name_ + "/microphone/" + "state" ;
        command_topic_ = prefix + "/" + name_ + "/microphone/" + "command";

        auto const &discoveryInfo = esphome::mqtt::global_mqtt_client->get_discovery_info();
        discovery_topic_ = discoveryInfo.prefix + "/sensor/" + name_ + "/" + name_ + "_microphone/config";

    }

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
        esphome::swarm_fft_audio::SwarmFFT *swarmFFT = getGlobalSwarmFFT();
        if(swarmFFT != NULL) {
            swarmFFT -> reportFFTResult(fft);
            ESP_LOGD(TAG, "FFT size: %d", fft.size());
            yield();
        } else {
            ESP_LOGE(TAG, "SwarmFFT instance is missing and NOT globally available.");
        }
    }

} // swarm_fft_audio namespace
} // esphome namespace
