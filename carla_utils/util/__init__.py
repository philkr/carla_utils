try:
    from tqdm import tqdm
except ImportError:
    def tqdm(x, *args, **kwargs):
        return x

from .util import Printable
from .client import add_argument as add_client_argument, from_args as client_from_args
