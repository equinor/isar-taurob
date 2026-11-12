import datetime

from pydantic import BaseModel, Field, FiniteFloat, field_validator


class PoseSnapshot(BaseModel):
    x: FiniteFloat
    y: FiniteFloat
    theta: FiniteFloat
    confidence: FiniteFloat | None = Field(default=None, ge=0, le=1)
    changed: datetime.datetime | None = None

    @field_validator("x", "y", "theta", "confidence", mode="before")
    @classmethod
    def reject_boolean_measurements(cls, value: object) -> object:
        if isinstance(value, bool):
            raise TypeError("Pose measurements must be numeric, not boolean")
        return value

    @property
    def source_timestamp(self) -> float | None:
        return self.changed.timestamp() if self.changed is not None else None
