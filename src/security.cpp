/**
 * From Vanetza (https://github.com/riebl/Vanetza)
 * Using Security V3
 *
 * https://github.com/riebl/vanetza/tree/master/vanetza/security
 * https://github.com/riebl/vanetza/tree/master/tools/socktap/security.cpp
 */

#include "autoware_v2x/security.hpp"

#include <memory>
#include <vanetza/security/delegating_security_entity.hpp>
#include <vanetza/security/straight_verify_service.hpp>
#include "vanetza/security/v2/certificate_provider.hpp"
#include <vanetza/security/v2/certificate_cache.hpp>
#include <vanetza/security/v2/default_certificate_validator.hpp>
#include <vanetza/security/v2/sign_service.hpp>
#include <vanetza/security/v3/certificate_cache.hpp>
#include <vanetza/security/v3/naive_certificate_provider.hpp>
#include <vanetza/security/v3/persistence.hpp>
#include <vanetza/security/v3/sign_header_policy.hpp>
#include <vanetza/security/v3/sign_service.hpp>
#include <vanetza/security/v3/static_certificate_provider.hpp>

#include <stdexcept>

using namespace vanetza;
namespace po = boost::program_options;

class SecurityContext : public security::SecurityEntity
{
public:
  SecurityContext(const Runtime &runtime, PositionProvider &positioning)
  : runtime(runtime),
    positioning(positioning),
    backend(security::create_backend("default")),
    sign_header_policy(runtime, positioning),
    cert_cache()
  {
  }

  security::EncapConfirm encapsulate_packet(security::EncapRequest &&request) override
  {
    if (!entity) {
      throw std::runtime_error("security entity is not ready");
    }
    return entity->encapsulate_packet(std::move(request));
  }

  security::DecapConfirm decapsulate_packet(security::DecapRequest &&request) override
  {
    if (!entity) {
      throw std::runtime_error("security entity is not ready");
    }
    return entity->decapsulate_packet(std::move(request));
  }

  void build_entity()
  {
    if (!cert_provider) {
      throw std::runtime_error("certificate provider is missing");
    }
    std::unique_ptr<security::SignService> sign_service{new security::v3::StraightSignService(*cert_provider, *backend, sign_header_policy)};
    std::unique_ptr<security::StraightVerifyService> verify_service{new security::StraightVerifyService(runtime, *backend, positioning)};
    verify_service->use_certificate_cache(&cert_cache);
    entity = std::make_unique<security::DelegatingSecurityEntity>(std::move(sign_service), std::move(verify_service));
  }

  const Runtime & runtime;
  PositionProvider & positioning;
  std::unique_ptr<security::Backend> backend;
  std::unique_ptr<security::SecurityEntity> entity;
  std::unique_ptr<security::v3::CertificateProvider> cert_provider;
  security::v3::DefaultSignHeaderPolicy sign_header_policy;
  security::v3::CertificateCache cert_cache;
};

std::unique_ptr<security::v3::CertificateProvider> load_certificates(
  const std::string &cert_path, const std::string &cert_key_path,
  const std::vector<std::string> &cert_chain_path, security::v3::CertificateCache &cert_cache)
{
  auto authorization_ticket = security::v3::load_certificate_from_file(cert_path);
  auto authorization_ticket_key = security::v3::load_private_key_from_file(cert_key_path);

  std::list<security::v3::Certificate> chain;
  for (auto & chain_path : cert_chain_path) {
    auto chain_certificate = security::v3::load_certificate_from_file(chain_path);
    chain.push_back(chain_certificate);
    cert_cache.store(chain_certificate);
  }

  return std::make_unique<security::v3::StaticCertificateProvider>(authorization_ticket, authorization_ticket_key.private_key, chain);
}

std::unique_ptr<security::SecurityEntity> create_security_entity(const po::variables_map &options, const Runtime &runtime, PositionProvider &positioning)
{
  std::unique_ptr<security::SecurityEntity> security;
  const std::string name = options["security"].as<std::string>();

  if (name.empty() || name == "none") {
    // no operation
  } else if (name == "certs") {
    if (options.count("certificate") ^ options.count("certificate-key")) {
      throw std::runtime_error("Either certificate and certificate-key must be present or none.");
    }

    if (options.count("certificate") && options.count("certificate-key")) {
      const auto &cert_path = options["certificate"].as<std::string>();
      const auto &cert_key_path = options["certificate-key"].as<std::string>();
      std::vector<std::string> chain_paths;
      if (options.count("certificate-chain")) {
        chain_paths = options["certificate-chain"].as<std::vector<std::string>>();
      }

      auto context = std::make_unique<SecurityContext>(runtime, positioning);
      context->cert_provider = load_certificates(cert_path, cert_key_path, chain_paths, context->cert_cache);
      context->build_entity();
      security = std::move(context);
    } else {
      auto context = std::make_unique<SecurityContext>(runtime, positioning);
      context->cert_provider = std::make_unique<security::v3::NaiveCertificateProvider>(runtime);
      context->build_entity();
      security = std::move(context);
    }

    if (!security) {
      throw std::runtime_error("internal failure setting up security entity");
    }
  } else {
    throw std::runtime_error("Unknown security entity requested");
  }

  return security;
}
