import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2s_audio
from esphome.const import CONF_ID

MULTI_CONF = False

AUTO_LOAD = ["sensor", "i2s_audio", "mqtt"]

DEPENDENCIES = ['i2s_audio', "mqtt"]

swarm_component_ns = cg.esphome_ns.namespace('swarm_audio')
SwarmFFTComponent = swarm_component_ns.class_("SwarmFFT",
                                              cg.Component)

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

CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(SwarmFFTComponent),
        }
    )
)
#).extend(cv.COMPONENT_SCHEMA).extend(i2s_audio.i2s_audio_component_schema())

#SWARMFFT_SCHEMA.extend(i2s.i2s_audio_device_schema())

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
#    await i2s.register_i2s_device(var, config)
