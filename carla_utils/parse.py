from .recording import parse

if __name__ == "__main__":
    # Testing the parsing function
    from argparse import ArgumentParser
    parser = ArgumentParser()
    parser.add_argument('recording')
    args = parser.parse_args()
    parse(args.recording)
