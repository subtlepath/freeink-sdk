// Network services and on-device OTA are intentionally unavailable in the
// offline browser demo. Return the real firmware's error states; never report
// fake successful transfers. Physical installation uses Web Serial instead.
#include "network/HttpDownloader.h"
#include "network/OtaUpdater.h"
#include "network/CrossPointWebServer.h"
#include "network/FirmwareFlasher.h"
#include <HalSystem.h>
void HalSystem::begin() {}
void HalSystem::checkPanic() {}
void HalSystem::clearPanic() {}
std::string HalSystem::getPanicInfo(bool) { return {}; }
bool HalSystem::isRebootFromPanic() { return false; }
bool HttpDownloader::fetchUrl(const std::string&, std::string&, const std::string&, const std::string&) { return false; }
bool HttpDownloader::fetchUrl(const std::string&, Stream&, const std::string&, const std::string&) { return false; }
bool HttpDownloader::fetchUrl(const std::string&, const DataCallback&, const std::string&, const std::string&) { return false; }
HttpDownloader::DownloadError HttpDownloader::downloadToFile(const std::string&, const std::string&, ProgressCallback, bool*, const std::string&, const std::string&, bool) { return HTTP_ERROR; }
bool OtaUpdater::isUpdateNewer() const { return false; }
const std::string& OtaUpdater::getLatestVersion() const { return latestVersion; }
OtaUpdater::OtaUpdaterError OtaUpdater::checkForUpdate() { return HTTP_ERROR; }
OtaUpdater::OtaUpdaterError OtaUpdater::installUpdate(ProgressCallback, void*) { return HTTP_ERROR; }
CrossPointWebServer::CrossPointWebServer() = default;
CrossPointWebServer::~CrossPointWebServer() = default;
void CrossPointWebServer::begin() {}
void CrossPointWebServer::stop() {}
void CrossPointWebServer::handleClient() {}
CrossPointWebServer::WsUploadStatus CrossPointWebServer::getWsUploadStatus() const { return {}; }
namespace firmware_flash {
Result flashFromSdPath(const char*, ProgressCb, void*, bool) { return Result::NO_PARTITION; }
Result validateImageFile(const char*, size_t) { return Result::NO_PARTITION; }
const char* resultName(Result) { return "On-device OTA is unavailable in the browser demo"; }
uint16_t runningPartitionChipId() { return 0xffff; }
}
