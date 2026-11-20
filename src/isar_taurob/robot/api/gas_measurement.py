import math
from decimal import Decimal

from pydantic import BaseModel, Field, RootModel, field_validator, model_validator
from robot_interface.models.inspection.inspection import (
    CO2Measurement,
    COMeasurement,
    GasMeasurement,
    H2SMeasurement,
    IsobutyleneMeasurement,
    O2Measurement,
    TolueneMeasurement,
)

# The keys are the species names reported by the X-am 8000 sensor.
GAS_MEASUREMENT_TYPES: dict[str, type[GasMeasurement]] = {
    "IBUT": IsobutyleneMeasurement,
    "CO2": CO2Measurement,
    "TOLU": TolueneMeasurement,
    "O2": O2Measurement,
    "CO": COMeasurement,
    "H2S": H2SMeasurement,
}


class _GasReading(BaseModel):
    value: Decimal = Field(strict=True, allow_inf_nan=False)
    unit: str

    @field_validator("value")
    @classmethod
    def validate_float_range(cls, value: Decimal) -> Decimal:
        if not math.isfinite(float(value)):
            raise ValueError("Gas reading must fit in a finite float")
        return value

    @property
    def decimal_places(self) -> int:
        return max(0, -int(self.value.as_tuple().exponent))


class GasMeasurements(RootModel[dict[str, _GasReading]]):
    @model_validator(mode="before")
    @classmethod
    def select_supported_readings(cls, payload: object) -> object:
        if not isinstance(payload, dict):
            return payload
        return {
            species: reading
            for species, reading in payload.items()
            if species in GAS_MEASUREMENT_TYPES and reading is not None
        }
