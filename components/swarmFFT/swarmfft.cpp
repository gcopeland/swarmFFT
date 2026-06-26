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
        audioCopier_(converter_, i2s_) {
    }

    void SwarmFFT::setup() {
        // Configuring audio toolkit logging
        ESP_LOGD(TAG, "SwarmFFT::setup() start");
        AudioLogger::instance().begin(Serial, AudioLogger::Info);

        // Make this instance globally available for static callbacks
        setGlobalSwarmFFT(this);
        ESP_LOGD(TAG, "SwarmFFT::setup() has registered for callbacks from static functions");

        // Configure MQTT
        auto *mqttClient = esphome::mqtt::global_mqtt_client;
        mqttClient->set_clean_session(false);
        //mqttClient->set_keep_alive(update_interval_/1000);
        
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
        i2sCfg.port_no = i2sPort_;
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
                                              }, 2, true);
        }
        
        return discoveryComplete_;
    }

    void SwarmFFT::on_json_message(const std::string &topic, JsonObject root) {
        size_t buffSize = measureJson(root) + 1;
        char jsonBuffer[buffSize];
        serializeJson(root, jsonBuffer, buffSize);
        ESP_LOGD(TAG, "MQTT on_json_message: %s", jsonBuffer);

        // Streaming Config
        // FIXME: WARNING - We are not doing input validation - this is dangerous
        auto sc = root["AudioConfig"];
        auto host = sc["host"].as<String>();
        auto port = sc["port"].as<uint16_t>();
        setVolume(sc["volume"].as<uint8_t>());
        setDBFloor(sc["db_floor"].as<float>());
        setSamplesPerMinute(sc["fftp_samples_per_minute"].as<uint16_t>());
        streamingEnabled_ = sc["enabled"].as<bool>();
    }

    // Verify the json object is a streamConfig with
    // all keys and value types
    bool SwarmFFT::isJSONStreamConfig(JsonObject &obj) {
        return true;
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

        // Move around our audio data for processing
        audioCopier_.copy();

        // Process FFT data if it's available
        if(haveFFTResult()) {
            processFFTResult();
        }
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
        ESP_LOGCONFIG(TAG, "   DB Floor Threshold: %f", dbFilterThreshold_);
        ESP_LOGCONFIG(TAG, "   MQTT State Topic    : %s", state_topic_.c_str());
        ESP_LOGCONFIG(TAG, "   MQTT Command Topic  : %s", command_topic_.c_str());
        ESP_LOGCONFIG(TAG, "   MQTT Discovery Topic: %s", discovery_topic_.c_str());
    }


    void SwarmFFT::reportFFTResult(AudioFFTBase &fft) {
        bool doUpdate = false;
        bool connected = is_connected();
        
        if(last_update_ == 0) {
            last_update_ = millis();
        }

        // Time to sample again?
        if(connected && update_interval_ > 0 &&
           millis() - last_update_  > update_interval_) {
            last_update_ = millis();
            doUpdate = true;
        }

        // If in continuous mode, force check
        if(connected && update_interval_ == 0) {
            doUpdate = true;
        }

        // If update needed, do it
        if(doUpdate) {
            if(haveFFTResult_) {
                ESP_LOGE(TAG, "FFT delivered with pending FFT.");
                fft_.reset();
            } else {
                fft.resultArray(fftResult_);
                haveFFTResult_ = true;
            }
        } else {
            // Clear the flag and reset the FFT buffers
            fft_.reset();
            haveFFTResult_ = false;
        }
    }


    /**
     * Walk the FFT bins starting at the provided index.
     * Return the first index which passes the filter. If
     * end is reached, return -1.
     */
    size_t SwarmFFT::filterToIndex(size_t startIndex) {
        size_t retValue = -1;

        // Make sure we don't look past
        if( startIndex >= FFT_BINS ) {
            return retValue;
        }

        for( size_t index=startIndex; index < FFT_BINS; ++index ) {
            auto bin = fftResult_[index];
            auto freq = bin.frequency;
            auto mag = bin.magnitude;
            auto db = 20 * log10(bin.magnitude);
            
            if(freq >= FFT_FILTER_MIN_FREQ &&
               freq <= FFT_FILTER_MAX_FREQ &&
               db >= dbFilterThreshold_) {
                retValue = index;
                break;
            }
        }

        return retValue;
    }

    
    void SwarmFFT::processFFTResult() {
        /**
         * As long as we are connected, loop through all FFT bins
         * Only add up to dataLength bins for each published json
         * message (stripe).
         */
        if( is_connected() ) {
            size_t binIndex = 0;
            size_t binSequence = 0;
            size_t dataLength = FFT_BINS/MQTT_FFT_STRIPES;

            bool haveData = false;
            bool haveError = false;
        
            for( size_t stripe=0; stripe < MQTT_FFT_STRIPES; ++stripe) {
                JsonDocument stripeDoc;
                stripeDoc[STRIPE_NAME] = name_;
                stripeDoc[STRIPE_STRIPE] = stripe;
                JsonArray data = stripeDoc[STRIPE_DATA].to<JsonArray>();

                // Loop through our bins and return the index of one which passes filter
                while( (binIndex = filterToIndex(binIndex)) != -1 ) {
                    // Have at least one bin so we need need to publish
                    haveData = true ;

                    // Populate our stripe's data array
                    JsonObject binDoc = data.add<JsonObject>();
                    binDoc[BIN_BIN] = ++binSequence;
                    binDoc[BIN_FREQ] = uint32_t(fftResult_[binIndex].frequency);
                    binDoc[BIN_MAG] = fftResult_[binIndex].magnitude;
                    binDoc[BIN_DB] = 20 * log10(fftResult_[binIndex].magnitude);

                    // Increment our bin index so the starting filter index is incredmented past tihs one
                    ++binIndex;
                
                    // If this filled the last slot in the stripe, break
                    // so we can send it
                    if( binSequence % dataLength == 0 ) {
                        break;
                    }
                }
                
                // We've processed at least one stripe, so send it if we have data
                if( haveData ) {
                    // Reset haveData to prevent empty stripe transmission next go around
                    haveData = false;

                    haveError = !publish_json(state_topic_, [stripeDoc](JsonObject msg) {
                        msg.set(stripeDoc.as<JsonObjectConst>());
                    }, 2, false);
                
                    if( haveError ) {
                        ESP_LOGD(TAG, "MQTT publish failed!");
                        break;
                    } else {
                        ESP_LOGD(TAG, "stripe successfully published");
                        // // Just for debugging
                        // String msg;
                        // serializeJson(stripeDoc, msg);
                        // ESP_LOGD(TAG, "published (%s): %s", state_topic_.c_str(), msg.c_str());
                    }
                }

                // If we failed to send, bail from this loop
                if( haveError ) {
                    ESP_LOGE(TAG, "Have error so bailing on the rest of the stripes");
                    break;
                }
            }
        } else {
            // Not connected so result the FFT until we are
            ESP_LOGD(TAG, "not connected so resetting FFT");
            fft_.reset();
        }

        // Reset our state for the next FFT collection
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
            // ESP_LOGD(TAG, "FFT size: %d", fft.size());
        } else {
            ESP_LOGE(TAG, "SwarmFFT instance is missing and NOT globally available.");
        }
    }

} // swarm_fft_audio namespace
} // esphome namespace
