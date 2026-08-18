from tests.utilities.mock_api_return_values.mock_taurob_api import MockTaurobAPI


class MockAPI:
    @staticmethod
    def schedule(instance_id=232):
        return instance_id

    @staticmethod
    def get_mission_instance_details(instance_id=232):
        return MockTaurobAPI.get_mission_details(instance_id=instance_id)

    @staticmethod
    def get_url_data():
        return MockTaurobAPI.get_url_data()

    @staticmethod
    def get_scheduled_actions():
        return [300]

    @staticmethod
    def delete_all_mission_instances():
        return

    @staticmethod
    def set_measurement_metadata():
        return

    @staticmethod
    def amend_mission():
        return

    @staticmethod
    def create_action(action_id=300):
        return action_id
