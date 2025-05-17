import esphome.codegen as cg

import esphome.config_validation as cv
from esphome.const import CONF_ID

MULTI_CONF = False
AUTO_LOAD = ["mqtt"]
DEPENDENCIES = ["mqtt"]

swarm_component_ns = cg.esphome_ns.namespace('swarm_fft_audio')
SwarmFFTComponent = swarm_component_ns.class_("SwarmFFT",
                                              cg.Component)

CONF_MIC_WS_PIN = "mic_ws_pin"
CONF_DATA_PIN = "i2s_din_pin"
CONF_CLOCK_PIN = "i2s_clock_pin"
CONF_MQTT_TOPIC_PREFIX = "mqtt_topic_prefix"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SwarmFFTComponent),
            cv.Required(CONF_MIC_WS_PIN): cv.int_range(min=1, max=255),
            cv.Required(CONF_DATA_PIN): cv.int_range(min=1, max=255),
            cv.Required(CONF_CLOCK_PIN): cv.int_range(min=1, max=255),
            cv.Required(CONF_MQTT_TOPIC_PREFIX): cv.string("hives"),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    # We require Arduino Audio Tools for Mic & FFT
    cg.add_library(
        name="arduino-audio-tools",
        repository="https://github.com/pschatzmann/arduino-audio-tools",
        version=None
    )

    # We require ArduinoJSON for serialization of the FFT data
    cg.add_library(
        name="ArduinoJson",
        version=None
    )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.setWsPin(config[CONF_MIC_WS_PIN]))
    cg.add(var.setDataPin(config[CONF_DATA_PIN]))
    cg.add(var.setClockPin(config[CONF_CLOCK_PIN]))
    cg.add(var.setMqttTopicPrefix(config[CONF_MQTT_TOPIC_PREFIX]))
