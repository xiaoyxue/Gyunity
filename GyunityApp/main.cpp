#include "Examples.h"
#include "SceneCommandLine.h"
#include <chrono>
#include <exception>
#include <iostream>

namespace {
int Run(int argc, const char* const* argv) {
	if (argc > 1) {
		const std::string command = argv[1];
		if (command == "--list-examples" && argc == 2) {
			for (const auto& name : Examples::Names()) std::cout << name << '\n';
			return 0;
		}
		if (command == "--example" || command == "--list-examples") {
			if (command == "--example" && argc == 3) return Examples::Named(argv[2]) ? 0 : 1;
			std::cerr << "GyunityApp: use --list-examples or --example NAME with default settings.\n";
			return 1;
		}
		return RunSceneCommandLine(argc, argv);
	}
	const auto begin = std::chrono::steady_clock::now();
	if (!Examples::PathTracing()) return 1;
	const std::chrono::duration<double, std::ratio<60>> elapsed = std::chrono::steady_clock::now() - begin;
	std::cout << "Cost time: " << elapsed.count() << " min\n";
	return 0;
}
}

int main(int argc, char* argv[]) {
	try {
		return Run(argc, argv);
	}
	catch (const std::exception& error) {
		std::cerr << "GyunityApp: unexpected failure: " << error.what() << '\n';
		return 1;
	}
}
