import esphome.codegen as cg

import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ID
from esphome.components.esp32 import add_idf_sdkconfig_option, include_builtin_idf_component

MULTI_CONF = False
AUTO_LOAD = ["mqtt",]
DEPENDENCIES = ["mqtt",]

# Setup our namespace and class
swarm_component_ns = cg.esphome_ns.namespace('swarm_fft_audio')
SwarmFFTComponent = swarm_component_ns.class_("SwarmFFT",
                                              cg.Component)

CONF_MIC_WS_PIN = "mic_ws_pin"
CONF_DATA_PIN = "i2s_din_pin"
CONF_CLOCK_PIN = "i2s_clock_pin"
CONF_I2S_PORT = "i2s_port"
CONF_DEFAULT_VOLUME = "default_volume"
CONF_DB_FLOOR = "db_floor"
CONF_DEVICE_NAME = "device_name"
CONF_MQTT_TOPIC_PREFIX = "mqtt_topic_prefix"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SwarmFFTComponent),
            cv.Required(CONF_MIC_WS_PIN): pins.internal_gpio_output_pin_number,
            cv.Required(CONF_DATA_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_CLOCK_PIN): pins.internal_gpio_input_pin_number,
            cv.Required(CONF_I2S_PORT): cv.int_range(0,1),
            cv.Required(CONF_DEFAULT_VOLUME): cv.float_range(0,1),
            cv.Required(CONF_DB_FLOOR): cv.float_range(-60,0),
            cv.Required(CONF_DEVICE_NAME): cv.string,
            cv.Required(CONF_MQTT_TOPIC_PREFIX): cv.string,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    
    # We require Arduino Audio Tools for Mic & FFT
    cg.add_library(
        name="arduino-audio-tools",
        repository="https://github.com/pschatzmann/arduino-audio-tools.git",
        version=None
    )

    # Add our audio processing libraries
    # cg.add_library(
    #     name="Ogg-codec",
    #     repository="https://github.com/pschatzmann/codec-ogg",
    #     version=None
    # )

    # # Add our audio processing libraries
    # cg.add_library(
    #     name="Opus-codec",
    #     repository="https://github.com/pschatzmann/codec-opus",
    #     version=None
    # )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.setWsPin(config[CONF_MIC_WS_PIN]))
    cg.add(var.setDataPin(config[CONF_DATA_PIN]))
    cg.add(var.setClockPin(config[CONF_CLOCK_PIN]))
    cg.add(var.setDeviceName(config[CONF_DEVICE_NAME]))
    cg.add(var.setMqttTopicPrefix(config[CONF_MQTT_TOPIC_PREFIX]))

    # include I2S configuration for ISR and the actual i2s driver
    add_idf_sdkconfig_option("CONFIG_I2S_ISR_IRAM_SAFE", True)

    # Re-enable ESP-IDF's I2S driver (excluded by default to save compile time)
    include_builtin_idf_component("esp_driver_i2s")
