from pisa_api.av import serve_av_system
from pisa_api.wrapper import setup_logging

from pcla_wrapper.pcla_av import PclaAV
from pcla_wrapper.version import wrapper_version

WRAPPER_NAME = "pcla-wrapper"

setup_logging()


if __name__ == "__main__":
    serve_av_system(PclaAV(), name=WRAPPER_NAME, version=wrapper_version())
