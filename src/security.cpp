#include "autoware_v2x/security.hpp"

#include <vanetza/security/delegating_security_entity.hpp>
#include <vanetza/security/straight_verify_service.hpp>
#include <vanetza/security/v2/certificate_cache.hpp>
#include <vanetza/security/v2/default_certificate_validator.hpp>
#include <vanetza/security/v2/naive_certificate_provider.hpp>
#include <vanetza/security/v2/null_certificate_validator.hpp>
#include <vanetza/security/v2/persistence.hpp>
#include <vanetza/security/v2/sign_header_policy.hpp>
#include <vanetza/security/v2/sign_service.hpp>
#include <vanetza/security/v2/static_certificate_provider.hpp>
#include <vanetza/security/v2/trust_store.hpp>

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
    cert_cache(runtime),
    cert_validator(*backend, cert_cache, trust_store)
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
    std::unique_ptr<security::SignService> sign_service = std::make_unique<security::v2::StraightSignService>(
      *cert_provider, *backend, sign_header_policy
    );
    std::unique_ptr<security::VerifyService> verify_service = std::make_unique<security::StraightVerifyService>(
      runtime, *backend, positioning
    );
    entity.reset(new security::DelegatingSecurityEntity{std::move(sign_service), std::move(verify_service)});
  }

  const Runtime &runtime;
  PositionProvider &positioning;
  std::unique_ptr<security::Backend> backend;
  std::unique_ptr<security::SecurityEntity> entity;
  std::unique_ptr<security::v2::CertificateProvider> cert_provider;
  security::v2::DefaultSignHeaderPolicy sign_header_policy;
  security::v2::TrustStore trust_store;
  security::v2::CertificateCache cert_cache;
  security::v2::DefaultCertificateValidator cert_validator;
};

std::unique_ptr<security::SecurityEntity>
create_security_entity(const Runtime &runtime, PositionProvider &positioning)
{
  std::unique_ptr<security::SecurityEntity> security;

  return security;
}
