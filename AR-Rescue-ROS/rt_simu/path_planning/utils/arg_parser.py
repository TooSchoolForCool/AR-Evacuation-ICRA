import argparse

def arg_parser():
    """Argument Parser
    Parse arguments from command line, and perform error checking
    Returns:
        An argument object which contains arguments from cmd line
    """
    parser = argparse.ArgumentParser(prog='Multi Agent Path Planning')

    parser.add_argument(
        "--map",
        dest="map",
        type=str,
        required=True,
        help="Map file directory"
    )
    parser.add_argument(
        "--agent",
        dest="agent",
        required=True,
        help="Agent location"
    )
    
    args = parser.parse_args()

    return args