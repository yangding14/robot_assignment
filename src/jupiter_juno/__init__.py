# Jupiter Juno Robot Package 

# Make sure the package can be imported as a module
__version__ = "1.0.0"
__author__ = "Jupiter Juno Team"

# Import key components for easier access
try:
    from . import jupiter_juno_monitor
    __all__ = ['jupiter_juno_monitor']
except ImportError:
    # Graceful fallback if Qt components aren't available
    __all__ = [] 