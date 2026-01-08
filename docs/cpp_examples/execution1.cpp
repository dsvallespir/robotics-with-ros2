#include <cstdio>
#include <execution>
#include <string>
#include <thread>
#include <utility>

using namespace std::literals;

int main()
{
    std::execution::run_loop lop;

    std::jthread worker([&](std::stop_token st)
    {
        std::stop_callback cb{st, [&]{ loop.finish();}} ;
        loop.run();
    });

    std::execution::sender auto hello = std::execution::just("hello world"s);
    std::execution::sender auto print
        = std::move(hello)
        | std::execution::then([](std::string msg)
        {
            return std::puts(msg.c_str());
        });

    std::execution::scheduler auto io_thread = loop.get_scheduler();
    std::execution::sender auto work = std::execution::on(io_thread, std::move(print));

    auto [result] = std::this_thread::sync_wait(std::move(work)).value();

    return result;
}