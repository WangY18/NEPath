#include <NEPath/IqopBuildIdentity.h>
#include <NEPath/IqopSolverDefaults.h>
#include <NEPath/IqopSubproblem.h>

#include <array>
#include <cstdint>
#include <iomanip>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#ifndef NEPATH_IFOPT_REVISION
#define NEPATH_IFOPT_REVISION "unavailable"
#endif

namespace
{
constexpr const char *FORMULATION_VERSION = "iqop-scp-v1";

// SHA-256 initial values and round constants from FIPS PUB 180-4.
constexpr std::array<std::uint32_t, 8> INITIAL_HASH{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                                                    0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
constexpr std::array<std::uint32_t, 64> ROUND_CONSTANTS{
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

[[nodiscard]] constexpr std::uint32_t rotate_right(std::uint32_t value, unsigned int count) noexcept
{
    return (value >> count) | (value << (32U - count));
}

[[nodiscard]] std::string sha256(std::string_view input)
{
    std::vector<std::uint8_t> message(input.begin(), input.end());
    const std::uint64_t bit_length = static_cast<std::uint64_t>(message.size()) * 8U;
    message.push_back(0x80U);
    while (message.size() % 64U != 56U)
    {
        message.push_back(0U);
    }
    for (int shift = 56; shift >= 0; shift -= 8)
    {
        message.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xffU));
    }

    std::array<std::uint32_t, 8> hash = INITIAL_HASH;
    for (std::size_t block = 0; block < message.size(); block += 64U)
    {
        std::array<std::uint32_t, 64> words{};
        for (std::size_t i = 0; i < 16U; ++i)
        {
            const std::size_t offset = block + 4U * i;
            words[i] = (static_cast<std::uint32_t>(message[offset]) << 24U) | (static_cast<std::uint32_t>(message[offset + 1U]) << 16U) |
                       (static_cast<std::uint32_t>(message[offset + 2U]) << 8U) | static_cast<std::uint32_t>(message[offset + 3U]);
        }
        for (std::size_t i = 16U; i < words.size(); ++i)
        {
            const std::uint32_t sigma0 = rotate_right(words[i - 15U], 7U) ^ rotate_right(words[i - 15U], 18U) ^ (words[i - 15U] >> 3U);
            const std::uint32_t sigma1 = rotate_right(words[i - 2U], 17U) ^ rotate_right(words[i - 2U], 19U) ^ (words[i - 2U] >> 10U);
            words[i] = words[i - 16U] + sigma0 + words[i - 7U] + sigma1;
        }

        std::uint32_t a = hash[0];
        std::uint32_t b = hash[1];
        std::uint32_t c = hash[2];
        std::uint32_t d = hash[3];
        std::uint32_t e = hash[4];
        std::uint32_t f = hash[5];
        std::uint32_t g = hash[6];
        std::uint32_t h = hash[7];
        for (std::size_t i = 0; i < words.size(); ++i)
        {
            const std::uint32_t sum1 = rotate_right(e, 6U) ^ rotate_right(e, 11U) ^ rotate_right(e, 25U);
            const std::uint32_t choice = (e & f) ^ ((~e) & g);
            const std::uint32_t temporary1 = h + sum1 + choice + ROUND_CONSTANTS[i] + words[i];
            const std::uint32_t sum0 = rotate_right(a, 2U) ^ rotate_right(a, 13U) ^ rotate_right(a, 22U);
            const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
            const std::uint32_t temporary2 = sum0 + majority;
            h = g;
            g = f;
            f = e;
            e = d + temporary1;
            d = c;
            c = b;
            b = a;
            a = temporary1 + temporary2;
        }
        hash[0] += a;
        hash[1] += b;
        hash[2] += c;
        hash[3] += d;
        hash[4] += e;
        hash[5] += f;
        hash[6] += g;
        hash[7] += h;
    }

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const std::uint32_t value : hash)
    {
        output << std::setw(8) << value;
    }
    return output.str();
}

void append_double(std::ostringstream &output, double value)
{
    output << std::hexfloat << value << ';';
}
} // namespace

namespace nepath
{
IqopBuildIdentity build_iqop_identity(const IqopSubproblem &subproblem)
{
    std::ostringstream input;
    input.imbue(std::locale::classic());
    input << FORMULATION_VERSION << ';' << IQOP_SOLVER_CONFIGURATION_VERSION << ';' << NEPATH_IFOPT_REVISION << ';';
    append_double(input, IQOP_DESIRED_SOLVER_TOLERANCE);
    append_double(input, IQOP_ACCEPTABLE_TOLERANCE_MULTIPLIER);
    append_double(input, IQOP_ACCEPTABLE_CONSTRAINT_MULTIPLIER);
    input << IQOP_ACCEPTABLE_ITERATION_COUNT << ';';
    append_double(input, IQOP_INITIAL_INTERIOR_SLACK);
    append_double(input, IQOP_INTERIOR_PUSH);
    append_double(input, IQOP_INTERIOR_FRACTION);
    input << IQOP_INNER_ITERATIONS_PER_SCP_STEP << ';';
    append_double(input, IQOP_INNER_WALL_TIME_SECONDS);
    append_double(input, IQOP_SCP_WALL_TIME_SECONDS);
    const NonEquidistantOptions &options = subproblem.options();
    append_double(input, options.delta);
    append_double(input, options.alpha);
    append_double(input, options.dot_delta);
    append_double(input, options.ddot_delta);
    input << options.optimize_Q << ';' << options.optimize_S << ';' << options.optimize_L << ';';
    append_double(input, options.lambda_Q);
    append_double(input, options.lambda_S);
    append_double(input, options.lambda_L);
    append_double(input, options.epsilon);
    input << options.step_max << ';';
    for (std::size_t i = 0; i < subproblem.geometry().vertex_count(); ++i)
    {
        append_double(input, subproblem.geometry().point(i).x.value());
        append_double(input, subproblem.geometry().point(i).y.value());
    }
    for (const double reference : subproblem.reference_offsets().values)
    {
        append_double(input, reference);
    }
    return IqopBuildIdentity{FORMULATION_VERSION, IQOP_SOLVER_CONFIGURATION_VERSION, NEPATH_IFOPT_REVISION, sha256(input.str())};
}
} // namespace nepath
