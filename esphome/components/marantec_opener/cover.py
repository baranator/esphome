import esphome.codegen as cg
from esphome.components import cover, uart
import esphome.config_validation as cv

marantec_opener_ns = cg.esphome_ns.namespace("marantec_opener")
MarantecOpener = marantec_opener_ns.class_("MarantecOpener", cover.Cover, cg.Component)

CONFIG_SCHEMA = (
    cover.cover_schema(MarantecOpener)
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "marantec_opener",
    baud_rate=1000,
    require_tx=True,
    require_rx=True,
    data_bits=8,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = await cover.new_cover(config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
