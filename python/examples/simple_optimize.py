#!/usr/bin/env python3

import argparse

import g2opy as g2o


def to_io_format(value: str) -> g2o.IoFormat:
    format = g2o.IoWrapper.format_for_file_extension(value)
    return format if format is not None else g2o.IoFormat.G2O


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=str, help="input filename")
    parser.add_argument(
        "-i", "--max_iterations", type=int, default=10, help="perform n iterations"
    )
    parser.add_argument(
        "-o", "--output", type=str, default="", help="save resulting graph as file"
    )
    parser.add_argument(
        "--output-format",
        type=str,
        choices=["g2o", "json", "bin"],
        default=g2o.IoFormat.G2O.name,
        help="define the output format",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="activate verbose output",
    )
    args = parser.parse_args()

    optimizer = g2o.SparseOptimizer()
    optimizer.set_verbose(args.verbose)
    optimizer.set_algorithm(
        g2o.OptimizationAlgorithmLevenberg(g2o.BlockSolverX(g2o.LinearSolverEigenX()))
    )

    print(f"Loading {args.input}")
    optimizer.load(args.input)
    print("Num vertices:", len(optimizer.vertices()))
    print("Num edges:", len(optimizer.edges()))
    optimizer.initialize_optimization()
    optimizer.compute_active_errors()
    print(f"Initial chi2: {optimizer.chi2()}")

    if args.max_iterations > 0:
        optimizer.optimize(args.max_iterations)
        optimizer.compute_active_errors()
        print(f"Final chi2: {optimizer.chi2()}")

    if len(args.output) > 0:
        io_output_format = to_io_format(args.output_format)
        print(f"Saving to {args.output} as {g2o.IoWrapper.to_string(io_output_format)}")
        optimizer.save(args.output, io_output_format)


if __name__ == "__main__":
    main()
