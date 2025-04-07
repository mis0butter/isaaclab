import argparse

# Create an ArgumentParser object
parser = argparse.ArgumentParser(description='A simple example program')

# Add arguments
parser.add_argument('-n', type=str, help='Your name')
parser.add_argument('-a', type=int, help='Your age')
parser.add_argument('-v', action='store_true', help='Increase output verbosity')

# Parse the arguments
args = parser.parse_args()

# Use the arguments
if args.v:
    print(f"Hello {args.n}, you are {args.a} years old!")
else:
    print(f"Hello {args.n}!")