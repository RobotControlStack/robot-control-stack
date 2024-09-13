import os

from dotenv import load_dotenv


def load_creds_fr3_desk() -> tuple[str, str]:
    load_dotenv()
    assert "DESK_USERNAME" in os.environ, "DESK_USERNAME not set in .env file or environment var."
    assert "DESK_PASSWORD" in os.environ, "DESK_PASSWORD not set in .env file or environment var."
    return os.environ["DESK_USERNAME"], os.environ["DESK_PASSWORD"]
