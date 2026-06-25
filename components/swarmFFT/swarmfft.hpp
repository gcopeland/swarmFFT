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

#include <cstdint>
#include <string>
#include <stdint.h>

#include <esphome.h>
#include <esphome/core/hal.h>  // for GPIOPin

// Audio processing & FFT
#include <AudioTools.h>
#include <AudioTools/AudioLibs/AudioRealFFT.h>

// Headers for audio streaming -  UDP + Opus + Ogg container
// #include <AudioTools/Communication/UDPStream.h>
// #include <AudioTools/AudioCodecs/CodecOpusOgg.h>
/*
** These must be declared in the including configuration
** file. This allows for different nodes to have different
** pins.
*/
namespace esphome {
namespace swarm_fft_audio {
    // Our logging component name
    static const char *TAG = "SwarmFFT";

    // Config constants
    // AUDIO INPUT
    static const uint16_t INPUT_CHANNELS = 2;
    static const uint16_t INPUT_BITS_PER_SAMPLE = 32;
    static const uint16_t INPUT_SAMPLES_PER_SECOND = 22050;

    // AUDIO OUTPUT
    static const uint16_t OUTPUT_CHANNELS = 1;
    static const uint16_t OUTPUT_BITS_PER_SAMPLE = 16;
    // static const uint16_t OUTPUT_SAMPLES_PER_SECOND = 11025;
    static const uint16_t OUTPUT_SAMPLES_PER_SECOND = INPUT_SAMPLES_PER_SECOND;

    // OPUS SPECIFIC
    // static const uint32_t OPUS_SAMPLE_RATE = 12000;
    // static const uint32_t OPUS_COMPEXITY = 3;

    // FFT SPECIFICS
    static const uint16_t SAMPLE_LENGTH = 256;
    static const uint16_t MIN_FREQ_THRESHOLD = 20.0;
    static const uint16_t MAX_FREQ_THRESHOLD = 1200.0;
    static const uint16_t FFT_BINS = SAMPLE_LENGTH / 2;
    static const uint16_t MQTT_FFT_STRIPES = 1;

    // FFT FILTER
    static const float FFT_FLOOR_THRESHOLD = -25.0;
    static const float FFT_FILTER_MIN_FREQ = 80.0;
    static const float FFT_FILTER_MAX_FREQ = 1200.0;


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
            bool doDiscovery();

            void on_json_message(const std::string &topic, JsonObject root);

            void on_safe_shutdown() override;

            void loop() override;

            void dump_config() override;

            bool haveFFTResult() const { return haveFFTResult_; }

            void reportFFTResult(AudioFFTBase &fft);

            // We only want to start after we have a connection else
            // it doesn't matter as we can't send it anyways.
            float get_setup_priority() const override {
                return esphome::setup_priority::DATA;
            }

            void setWsPin(uint8_t wsPin) { wsPin_ = wsPin; }
            void setDataPin(uint8_t dataPin) { dataPin_ = dataPin; }
            void setClockPin(uint8_t clockPin) { clockPin_ = clockPin; }
            void setDeviceName(std::string name);
            void setMqttTopicPrefix(std::string prefix);
            void setI2SPort(uint16_t port) { i2sPort_ = port; }
            void setVolume(float volume) { audioLevel_ = volume; }
            void setDBFloor(float floorDB) { dbFilterThreshold_ = floorDB; }

            // Zero means continuous
            void setSamplesPerMinute(uint32_t interval) { update_interval_ = (interval > 0)?60*1000/interval : 0; };
            
        private:
            I2SStream i2s_;
            I2SConfig cfg_;
            StreamCopy audioCopier_;
            // WiFiUDP udp_;
            // UDPStream udpStream_;
            AudioRealFFT fft_;
            MultiOutput multi_;           // multiple destinations
            // Throttle throttle_;           // throttle our stream data
            VolumeStream volume_;         // Dynamically adjust volume
            // MeasuringStream measure_;     // measure out output to our datagrams
            FormatConverterStream converter_; // convert is2 -> 16bit mono
            // OpusAudioEncoder audioEncoder_;
            // EncodedAudioStream encodedAudioStream_;
            //FilteredStream<uint16_t, BandPassFilter<float>> bandPass_; //Filter our input

            bool haveFFTResult_ = false;
            bool streamingEnabled_ = false;
            bool discoveryComplete_ = false;
            float audioLevel_ = 1.0f;
            float dbFilterThreshold_ = -120.0f;
            uint16_t i2sPort_ = 0;
            uint32_t lastCopy_ = 0;
            uint32_t last_update_ = 0;
            uint32_t update_interval_ = 1;
            AudioFFTResult fftResult_[FFT_BINS] = {0};

            // Process our FFT audio data
            bool filterByIndex(size_t index);
            void processFFTResult();
            bool isJSONStreamConfig(JsonObject &obj);
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
