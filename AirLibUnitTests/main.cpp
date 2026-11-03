#include "SettingsTest.hpp"
#include "PixhawkTest.hpp"
#include "SimpleFlightTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"
#include "CelestialTests.hpp"
#include "RovHydrodynamicsTest.hpp"
#include <iostream>

int main()
{
    using namespace msr::airlib;

    std::pair<std::string, std::unique_ptr<TestBase>> tests[] = {
        { "QuaternionTest", std::unique_ptr<TestBase>(new QuaternionTest()) },
        { "RovHydrodynamicsTest", std::unique_ptr<TestBase>(new RovHydrodynamicsTest()) }
    };

    int passed = 0;
    int failed = 0;

    for (auto& t : tests) {
        std::cout << "\n==========================================" << std::endl;
        std::cout << ">>> Running " << t.first << "..." << std::endl;
        std::cout << "==========================================" << std::endl;
        try {
            t.second->run();
            std::cout << ">>> " << t.first << " PASSED!" << std::endl;
            passed++;
        }
        catch (const std::exception& ex) {
            std::cerr << ">>> " << t.first << " FAILED: " << ex.what() << std::endl;
            failed++;
        }
    }

    std::cout << "\n==========================================" << std::endl;
    std::cout << "TEST SUMMARY: " << passed << " passed, " << failed << " failed." << std::endl;
    std::cout << "==========================================" << std::endl;

    return failed == 0 ? 0 : 1;
}



