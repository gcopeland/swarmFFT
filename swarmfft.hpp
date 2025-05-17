#pragma once

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

#include <AudioTools/AudioLibs/AudioRealFFT.h>
#include <AudioTools.h>

/*
** These must be declared in the including configuration
** file. This allows for different nodes to have different
** pins.
*/
namespace esphome {
namespace swarm_fft_audio {
    // Our logging component name
    static const char *TAG = "SwarmFFT";

    // Configs
    static const uint16_t CHANNELS = 2;
    static const uint16_t BITS_PER_SAMPLE = 32;
    static const uint16_t SAMPLE_LENGTH = 512;
    static const uint16_t MAX_FREQUENCY_HZ = 2000;
    static const uint16_t SAMPLES_PER_SECOND = MAX_FREQUENCY_HZ * 2;
    static const uint16_t FFT_BINS = SAMPLE_LENGTH / 2;


    class SwarmFFT : public esphome::Component,
                     public esphome::mqtt::CustomMQTTDevice {
        public:
            // Data pin
            uint8_t wsPin_;
            uint8_t clockPin_;
            uint8_t dataPin_;

            // Quick reference strings - constructed
            std::string name_;
            std::string prefix_;
            std::string state_topic_;
            std::string command_topic_;
            std::string discovery_topic_;

            // Constructor
            explicit SwarmFFT();

            // Setup all of the moving parts
            void setup() override;

            // Announce ourselves to home assistant
            void doDiscovery();

            void on_json_message(JsonObject root);

            void on_shutdown() override;

            void loop() override;

            bool haveFFTResult() const { return haveFFTResult_; }

            void reportFFTResult(AudioFFTBase &fft);

            // Tell it we are only data and can start later
            float get_setup_priority() const override {
                return esphome::setup_priority::AFTER_CONNECTION;
            }

            void setWsPin(uint8_t wsPin) { wsPin_ = wsPin; }
            void setDataPin(uint8_t dataPin) { dataPin_ = dataPin; }
            void setClockPin(uint8_t clockPin) { clockPin_ = clockPin; }
            void setMqttTopicPrefix(std::string prefix);

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
            bool discoveryComplete_ = false;

            // Process our FFT audio data
            void processFFTResult();
    };

    /*
    ** =======================================================
    ** Declare functions to handle our global reference to our
    ** swarm singleton and process our FFT results.
    */
    // Global access functions and state
    SwarmFFT *setGlobalSwarmFFT(SwarmFFT *swarmFFT);
    SwarmFFT *getGlobalSwarmFFT();
    static void fftCallback(AudioFFTBase &fft);

} // namespace swarm_audio
} // namespace esphome
