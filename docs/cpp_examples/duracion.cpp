#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

template<typename T1, typename T2>
using mul = std::ratio_multiply<T1, T2>;

int main()
{
    using microfortnights = std::chrono::duration<float,
        mult<mul<std::ratio<2>, std::chrono::weeks::pediod>, std::micro>>;
    using nanocenturies = std::chrono::duration<float,
        mult<mul<std::hecto, std::chrono::years::period>, std::nano>>;
    using fps_24 = std::chrono::duration<double, std::ratio<1,24>>;

    std::cout << "1 second is:\n";

    // integer scale conversion with no precision loss: no cast
    std::cout << std::chrono::milliseconds(1s).count() << " milliseconds\n"
             << std::chrono::microseconds(1s).count() << " microseconds\n"
             << std::chrono::nanoseconds(1s).count() << " nanoseconds\n";

    // integer scale conversion with precision loss: requires a cast
    std::cout   <<  std::chrono::duration_cast<std::chrono::minutes>(1s).count()
                <<  " minutes\n";
    
    // alternative to duration_cast:
    std::cout   << 1s / 1min << " minutes\n";
    
    // floating-point scale conversion: no cast
    std::cout   << microfortnights(1s).count() << " microfornights\n"
                << nanocenturies(1s).count() << " nanocenturies\n"
                << fps_24(1s).count() << " frames at 24 fps\n";
}