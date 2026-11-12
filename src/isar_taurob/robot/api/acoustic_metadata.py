from datetime import UTC
from typing import Literal

from pydantic import (
    AliasPath,
    AwareDatetime,
    BaseModel,
    Field,
    FiniteFloat,
    model_validator,
)
from robot_interface.models.inspection.inspection import (
    AcousticMeasurementMetadata,
    InspectionMetadata,
)

# The Taurob API records acoustic measurements for a fixed duration
ACOUSTIC_MEASUREMENT_DURATION: float = 2


class AcousticMetadata(BaseModel):
    version: Literal[1]
    type: Literal["leakDetection"]
    start_time: AwareDatetime = Field(alias="dateTime")
    snr_threshold: FiniteFloat = Field(alias="snrThreshold")
    snr_value: FiniteFloat = Field(alias="snrValue")
    leak_rate: FiniteFloat = Field(
        validation_alias=AliasPath("ldMetadata", "results", "rate"), ge=0
    )
    leak_rate_unit: str = Field(
        validation_alias=AliasPath("ldMetadata", "results", "rateUnit"),
        min_length=1,
        pattern=r"\S",
    )
    sound_pressure_level_at_source_db: FiniteFloat = Field(alias="splAtSource")
    distance_to_source: FiniteFloat = Field(alias="estimatedDistance", gt=0)
    frequency_from: FiniteFloat = Field(alias="frequencySelectionFrom", ge=0)
    frequency_to: FiniteFloat = Field(alias="frequencySelectionTo")
    distance_unit: Literal["meter"] = Field(alias="estimatedDistanceUnit")
    pressure_unit: Literal["dBSPL"] = Field(alias="splAtSourceUnit")
    frequency_from_unit: Literal["Hz"] = Field(alias="frequencySelectionFromUnit")
    frequency_to_unit: Literal["Hz"] = Field(alias="frequencySelectionToUnit")

    @model_validator(mode="after")
    def validate_frequency_range(self) -> AcousticMetadata:
        if self.frequency_to <= self.frequency_from:
            raise ValueError("Acoustic frequency range must be increasing")
        return self

    def to_inspection_metadata(
        self, context: InspectionMetadata
    ) -> AcousticMeasurementMetadata:
        return AcousticMeasurementMetadata(
            start_time=self.start_time.astimezone(UTC),
            robot_pose=context.robot_pose,
            target_position=context.target_position,
            file_type="mp4",
            tag_id=context.tag_id,
            inspection_description=context.inspection_description,
            duration=ACOUSTIC_MEASUREMENT_DURATION,
            snr_value=self.snr_value,
            leak_rate=self.leak_rate,
            leak_rate_unit=self.leak_rate_unit,
            sound_pressure_level_at_source_db=self.sound_pressure_level_at_source_db,
            distance_to_source=self.distance_to_source,
            result=(
                "RI_ANOMALY" if self.snr_value > self.snr_threshold else "RI_NORMAL"
            ),
            frequency_from=self.frequency_from,
            frequency_to=self.frequency_to,
        )
