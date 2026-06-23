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

#include <cstdint>

#include "swarmfft.hpp"

/*
** These must be declared in the including configuration
** file. This allows for different nodes to have different
** pins.
 */

namespace esphome {
namespace swarm_fft_audio {
    SwarmFFT::SwarmFFT() :
        audioCopier_(converter_, i2s_),
        wsPin_(0),
        clockPin_(0),
        dataPin_(0),
        lastCopy_(0),
        incompleteAudio_(true),
        update_interval_(0xFFFFFFFF) {
        ESP_LOGI(TAG, "ESPHhome prevents RAII");
    }

    void SwarmFFT::setup() {
        // Configuring audio toolkit logging
        ESP_LOGD(TAG, "SwarmFFT::setup() start");
        AudioLogger::instance().begin(Serial, AudioLogger::Info);

        // Make this instance globally available for static callbacks
        setGlobalSwarmFFT(this);
        ESP_LOGD(TAG, "SwarmFFT::setup() has registered for callbacks from static functions");

        // Subscribe to our MQTT command topic
        subscribe_json(command_topic_, &SwarmFFT::on_json_message);

        // Audio processing graph
        // i2s -> volume -> converter -> multi
        // multi^1 -> fft
        // multi^2 -> audio streaming (not implemented)

        // Common audio configuration
        auto audioFrom = AudioInfo(INPUT_SAMPLES_PER_SECOND, INPUT_CHANNELS, INPUT_BITS_PER_SAMPLE);
        auto audioTo = AudioInfo(OUTPUT_SAMPLES_PER_SECOND, OUTPUT_CHANNELS, OUTPUT_BITS_PER_SAMPLE);

        // Configure I2S input (INMP441, RX_MODE) ─────────────────────────────
        auto i2sCfg = i2s_.defaultConfig(RX_MODE);
        i2sCfg.copyFrom(audioFrom);
        i2sCfg.pin_ws   = wsPin_;
        i2sCfg.pin_bck  = clockPin_;
        i2sCfg.pin_data = dataPin_;
        i2sCfg.i2s_format = I2S_STD_FORMAT;   // standard Philips I2S
        i2sCfg.port_no = 0;  // FIXME: Make this a configuration
        i2sCfg.is_master = true;              // ESP32 drives the clock
        i2sCfg.use_apll = true;
        i2s_.begin(i2sCfg);

        // Increase audio level
        auto volumeCfg = volume_.defaultConfig();
        volumeCfg.copyFrom(audioFrom);
        volumeCfg.volume = audioLevel_;
        volume_.setOutput(converter_);
        volume_.begin(volumeCfg);

        // Converter (i2s -> 16b mono) -> multi
        converter_.setOutput(multi_);
        converter_.begin(audioFrom, audioTo);
        Serial.printf("FormatConverter started: %dch/%dbit/%dHz → %dch/%dbit/%dHz\n",
                      audioFrom.channels, audioFrom.bits_per_sample, audioFrom.sample_rate,
                      audioTo.channels, audioTo.bits_per_sample, audioTo.sample_rate);
        
        
        // Setup band pass filtering
        // BandPassFilter<float> bpFilter(, , info_.sample_rate);
        // auto filterCfg == bandPass_.defaultConfig();
        // bandPass_.setFilter(bpFilter);
        // bandPass_.begin(filterCfg);
        // bandPass_.setInput(volume_);

        auto fftCfg = fft_.defaultConfig();
        fftCfg.copyFrom(audioTo);           // expects the converted format
        fftCfg.length   = SAMPLE_LENGTH;
        fftCfg.stride   = SAMPLE_LENGTH;
        fftCfg.window_function = new BufferedWindow(new Hamming());
        fftCfg.callback = fftCallback; // result callback
        fft_.begin(fftCfg);
        multi_.add(fft_);
        
        // // Audio compression - Opus Codec
        // opusCfg_ = opus_.defaultConfig();
        // opusCfg_.copyFrom(info_);
        // opusCfg_.application = OPUS_APPLICATION_AUDIO;
        // opusCfg_.bitrate = 16000;
        // opusCfg_.complexity = 5;
        // opus_.begin(opusCfg_);

        // // UDP Throttling - keep pace with available
        // // data rather than sending as quickly as possible.
        // throttleCfg_ = throttle_.defaultConfig();
        // throttleCfg_.copyFrom(info_);
        // throttle_.begin(throttleCfg_);

        // // Set the opus stream to throttle
        // audioStream_.begin(throttle_, &audioStream_);
        multi_.begin(audioTo);

        // Setup our audio copier
        // i2s -> volume -> converter -> multi [fft, audio]
        //audioCopier_.begin(volume_, i2s_);

        ESP_LOGD(TAG, "SwarmFFT::setup() complete");
    }

    
    // Announce ourselves to home assistant
    bool SwarmFFT::doDiscovery() {
        if(!discoveryComplete_ && is_connected()) {
            // ESP_LOGD(TAG, "doDiscovery(), is connected but discovery is not complete.");
            discoveryComplete_ = publish_json(discovery_topic_,
                                              [this](JsonObject root) {
                                                  root["name"] = name_ + "_swarm_microphone_fft";
                                                  root["stat_cla"] = "measurement";
                                                  root["suggested_area"] = "hives";
                                                  root[esphome::mqtt::MQTT_STATE_TOPIC] = state_topic_;
                                                  root[esphome::mqtt::MQTT_COMMAND_TOPIC] = command_topic_;
                                                  root[esphome::mqtt::MQTT_JSON_ATTRIBUTES_TOPIC] = state_topic_;
                                                  root[esphome::mqtt::MQTT_UNIQUE_ID] = name_ + "_microphone_" + get_mac_address();
                                                  root[esphome::mqtt::MQTT_ICON] = "mdi:microphone-plus";
                                                  root[esphome::mqtt::MQTT_JSON_ATTRIBUTES_TEMPLATE] = "{\"bin\":\"{{value}}\"}";

                                                  JsonObject dev = root["dev"].to<JsonObject>();
                                                  dev[esphome::mqtt::MQTT_DEVICE_NAME] = name_;
                                                  dev[esphome::mqtt::MQTT_DEVICE_IDENTIFIERS] = "ESP_MICROPHONE_" + get_mac_address();
                                                  dev[esphome::mqtt::MQTT_DEVICE_SW_VERSION] = ESPHOME_VERSION;
                                                  dev[esphome::mqtt::MQTT_DEVICE_MANUFACTURER] = "gtcopeland";
                                              }, 0, true);
        }
        
        return discoveryComplete_;
    }

    void SwarmFFT::on_json_message(const std::string &topic, JsonObject root) {
        size_t buffSize = measureJson(root) + 1;
        char jsonBuffer[buffSize];
        serializeJson(root, jsonBuffer, buffSize);
        ESP_LOGD(TAG, "MQTT on_json_message: %s", jsonBuffer);

        // // Streaming Config
        // //
        // if(isJSONStreamConfig(root)) {
        //     auto sc = root["streamConfig"];
        //     auto host = sc["host"].as<String>();
        //     auto port = sc["port"].as<uint16_t>();
        //     auto volume = sc["volume"].as<uint8_t>();
        //     auto enabled = sc["enabled"].as<bool>();

        //     audioLevel_ = map(volume, 0, 255, 0.0, 8.0);
        //     streamingEnabled_ = enabled;
        // } else {
        //     ESP_LOGD(TAG, "streamConfig key not found");
        // }
    }

    // Verify the json object is a streamConfig with
    // all keys and value types
    bool SwarmFFT::isJSONStreamConfig(JsonObject &obj) {
        bool retValue = false;
        const char objName[] = "streamConfig";

        if(obj[objName].is<JsonObject>()) {
            auto sc = obj[objName];
            retValue = sc["host"].is<String>() && sc["port"].is<int>() &&
                sc["volme"].is<int>() && sc["enabled"].is<bool>();
        }

        return retValue;
    }

    void SwarmFFT::on_safe_shutdown() {
        // TODO: Update the command topic that streaming is not available
        //ESP_LOGI(TAG, "CHIPEN pin not exposed. Sleep mode not available.");
        ESP_LOGD(TAG, "on_safe_shutdown()");
    }


    void SwarmFFT::loop() {
        // Copy the data to our FFT which will trigger our FFT callback
        // for processing. This works by waiting for the update interval
        // to expire OR continuing copying until enough audio data is
        // available for FFT processing. Once enough data is copied, it
        // begins to wait for the update interval again.

        // Make sure have had HA discovery information
        doDiscovery();

        // Process FFT data if it's available
        if(haveFFTResult()) {
            incompleteAudio_ = false;
            processFFTResult();
            yield();
        }
        audioCopier_.copy();

    }

    void SwarmFFT::dump_config() {
        ESP_LOGCONFIG(TAG, "SwarmFFT:");
        ESP_LOGCONFIG(TAG, "   mic_ws_pin: %i", wsPin_);
        ESP_LOGCONFIG(TAG, "   i2s_din_pin: %i", dataPin_);
        ESP_LOGCONFIG(TAG, "   i2s_clock_pin: %i", clockPin_);
        ESP_LOGCONFIG(TAG, "   INPUT Channels: %i", INPUT_CHANNELS);
        ESP_LOGCONFIG(TAG, "   OUTPUT Channels: %i", OUTPUT_CHANNELS);
        ESP_LOGCONFIG(TAG, "   INPUT BPS: %i", INPUT_BITS_PER_SAMPLE);
        ESP_LOGCONFIG(TAG, "   OUPUT BPS: %i", OUTPUT_BITS_PER_SAMPLE);
        ESP_LOGCONFIG(TAG, "   INPUT SPS: %i", INPUT_SAMPLES_PER_SECOND);
        ESP_LOGCONFIG(TAG, "   OUTPUT SPS: %i", OUTPUT_SAMPLES_PER_SECOND);
        ESP_LOGCONFIG(TAG, "   FFT BINS: %i", FFT_BINS);
        ESP_LOGCONFIG(TAG, "   FFT SAMPLE SIZE: %i", SAMPLE_LENGTH);
        ESP_LOGCONFIG(TAG, "   MQTT FFT STRIPES: %i", MQTT_FFT_STRIPES);
        ESP_LOGCONFIG(TAG, "   Min Freq Threshold: %i", MIN_FREQ_THRESHOLD);
        ESP_LOGCONFIG(TAG, "   Max Freq Threshold: %i", MAX_FREQ_THRESHOLD);
        ESP_LOGCONFIG(TAG, "   DB Floor Threshold: %d", DB_FLOOR_THRESHOLD);
        ESP_LOGCONFIG(TAG, "   MQTT State Topic    : %s", state_topic_.c_str());
        ESP_LOGCONFIG(TAG, "   MQTT Command Topic  : %s", command_topic_.c_str());
        ESP_LOGCONFIG(TAG, "   MQTT Discovery Topic: %s", discovery_topic_.c_str());
    }


    void SwarmFFT::reportFFTResult(AudioFFTBase &fft) {
        if(haveFFTResult_) {
            ESP_LOGE(TAG, "FFT delivered with pending FFT.");
        } else {
            fft.resultArray(fftResult_);
            haveFFTResult_ = true;
        }
    }


    /*
     * Based upon freq range criteria, pass in an index
     * and it returns the index of the next fft result
     * which satisfies the filter criteria. Returns -1
     * if end of results is reached.
     */
    size_t SwarmFFT::filterToIndex(size_t startIndex) {
        size_t retValue = -1;

        for(size_t index=startIndex; index < fft_.size(); ++index) {
            auto bin = fftResult_[index];
            auto freq = bin.frequency;
            auto mag = bin.magnitude;
            auto db = 20 * log10(bin.magnitude);
            
            if(freq >= FFT_FILTER_MIN_FREQ &&
               freq <= FFT_FILTER_MAX_FREQ &&
               db >= FFT_FLOOR_THRESHOLD) {
                retValue = index;
                break;
            }
        }

        return retValue;
    }


    void SwarmFFT::processFFTResult() {
        if(is_connected()) {
            // Break our FFT data in stripes - MQTT_FFT_STRIPES count
            auto stripeLength = FFT_BINS/MQTT_FFT_STRIPES;
            for(auto stripe=0; stripe < MQTT_FFT_STRIPES; stripe++) {
                auto pubResult = false;
                while( pubResult == false ) {
                    
                    pubResult = 1 == publish_json(state_topic_,
                                                  [this, stripe](JsonObject root) {
                                                      auto stripeLength = FFT_BINS/MQTT_FFT_STRIPES;
                                                      root["node"] = name_;
                                                      root["stripe"] = stripe;
                                                      JsonArray dataDoc = root["data"].to<JsonArray>();

                                                      for(auto index=0; index < stripeLength; index++) {
                                                          auto bin = (stripe * stripeLength) + index;
                                                          auto freq = fftResult_[bin].frequency;
                                                          auto mag = fftResult_[bin].magnitude;
                                                          auto db = 20 * log10(fftResult_[bin].magnitude);
                                                          
                                                          // Create our bin and add it to the array of bins
                                                          JsonObject binDoc = dataDoc.add<JsonObject>();
                                                          binDoc["bin"] = bin;
                                                          binDoc["frequency"] = freq;
                                                          binDoc["magnitude"] = mag;
                                                          binDoc["db"] = db;
                                                      }
                                                    
                                                      String text;
                                                      serializeJson(root, text);
                                                      ESP_LOGD(TAG, "publish: %s", text.c_str());
                                                  }, 2, false);

                    if (!pubResult) {
                        // Allow us to bail if we lose connection
                        pubResult = is_connected();
                    } else {
                        yield();
                    }
                };
            }
            //     ESP_LOGD(TAG, "[%s] Published %d stripes of data to: %s", name_.c_str(),
            //              MQTT_FFT_STRIPES,
            //              state_topic_.c_str());

        } else {
            ESP_LOGD(TAG, "not connected. not sending data");
        }

        incompleteAudio_ = false;
        haveFFTResult_ = false;
    }


    void SwarmFFT::setDeviceName(std::string name) {
        // Set the device name - required for data to correctly register
        name_ = name;
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
    ** Declare functions to handle the global reference to our
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
        } else {
            ESP_LOGE(TAG, "SwarmFFT instance is missing and NOT globally available.");
        }
    }

} // swarm_fft_audio namespace
} // esphome namespace
