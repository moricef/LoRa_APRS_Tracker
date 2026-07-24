// Custom scripts

let currentSettings = null;
let editableLoraProfiles = [];
let editableWifiAPs = [];
let loraFrequencyLimits = { min: 100000000, max: 1000000000 };
const DEFAULT_LORA_PROFILES = [
    { profileName: "EU/WORLD", frequency: 433775000, signalBandwidth: 125000, spreadingFactor: 12, codingRate4: 5, power: 20 },
    { profileName: "Poland", frequency: 434855000, signalBandwidth: 125000, spreadingFactor: 9, codingRate4: 7, power: 20 },
    { profileName: "UK", frequency: 439912500, signalBandwidth: 125000, spreadingFactor: 12, codingRate4: 5, power: 20 }
];

function byId(id) { return document.getElementById(id); }
function setValue(id, value) { const element = byId(id); if (element) element.value = value ?? ""; }
function setChecked(id, value) { const element = byId(id); if (element) element.checked = Boolean(value); }
function setDisabled(id, value) { const element = byId(id); if (element) element.disabled = Boolean(value); }
function escapeHtml(value) { return String(value ?? "").replaceAll("&", "&amp;").replaceAll('"', "&quot;").replaceAll("<", "&lt;").replaceAll(">", "&gt;"); }
function numeric(value, fallback) { const parsed = Number(value); return Number.isFinite(parsed) ? parsed : fallback; }
function cloneLoraProfile(profile) { return { ...profile }; }
function applyBuildInfoPreviewFallback() {
    const element = byId("buildInfo");
    if (element && element.textContent.trim() === "%BUILD_INFO%") {
        // Historical upstream CA2RXU firmware version. Keep fixed; this fork uses LVGL UI versions.
        element.innerHTML = "Board / Environment = local preview<br>Firmware (CA2RXU)<br>Version: 2.4.1<br>Date: 2026-01-12<br>LVGL UI<br>Version: 2.9.6<br>Build date: generated during firmware build";
    }
}
function defaultLoraProfiles() {
    const profiles = DEFAULT_LORA_PROFILES.filter((profile) => profile.frequency >= loraFrequencyLimits.min && profile.frequency <= loraFrequencyLimits.max);
    return (profiles.length ? profiles : DEFAULT_LORA_PROFILES).map(cloneLoraProfile);
}

function backupSettings() {
    const data = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(currentSettings));
    const a = document.createElement("a");
    a.setAttribute("href", data);
    a.setAttribute("download", "TrackerConfigurationBackup.json");
    a.click();
}

function showToast(message) {
    const el = document.querySelector("#toast");
    el.querySelector(".toast-body").innerHTML = message;
    (new bootstrap.Toast(el)).show();
}

function fetchSettings() {
    fetch("/configuration.json")
        .then((response) => response.json())
        .then((settings) => loadSettings(settings))
        .catch((err) => { console.error(err); loadSettings(defaultPreviewSettings()); });
}

function defaultPreviewSettings() {
    return {
        beacons: [{
            callsign: "N0CALL-7",
            profileLabel: "MAIN",
            comment: "",
            status: "",
            symbol: ">",
            overlay: "/",
            micE: "",
            smartBeaconActive: true,
            gpsEcoMode: false,
            smartBeaconSetting: 2,
            sbSlowRate: 120,
            sbFastRate: 60,
            sbMinSpeed: 10,
            sbMaxSpeed: 70,
            sbMinTurnAngle: 28,
            sbTurnSlope: 240,
            sbMinBeaconTime: 10
        }],
        other: {},
        display: {},
        bluetooth: {},
        aprs_is: {},
        loraConfig: {},
        loraFreqMin: 100000000,
        loraFreqMax: 1000000000,
        lora: defaultLoraProfiles(),
        battery: {},
        telemetry: {},
        winlink: {},
        wifi: { AP: [{}], autoAP: {} },
        notification: {},
        pttTrigger: {}
    };
}

function renderSmartBeaconInput(index, field, label, value) {
    return '<div class="col-6 mb-1"><label class="form-label small mb-0">' + label + '</label><input type="number" class="form-control form-control-sm" name="beacons.' + index + '.' + field + '" id="beacons.' + index + '.' + field + '" min="0" step="1" value="' + value + '"></div>';
}

function renderBeaconProfiles(beacons) {
    const container = byId("beacon-settings");
    container.innerHTML = "";
    const stationProfiles = Array.from({ length: 3 }, (_, index) => {
        return (beacons || [])[index] || {
            callsign: "",
            profileLabel: "PROFILE " + (index + 1),
            comment: "",
            status: "",
            symbol: ">",
            overlay: "/",
            micE: "",
            smartBeaconActive: false,
            gpsEcoMode: false,
            smartBeaconSetting: 2
        };
    });
    stationProfiles.forEach((beacon, index) => {
        const element = document.createElement("div");
        element.classList.add("row", "beacons", "border-bottom", "py-2");
        element.innerHTML = '<div class="beacon-row w-100">' +
            '<div class="list-index"><strong>' + (index + 1) + ')</strong></div>' +
            '<div class="form-floating field-callsign"><input type="text" class="form-control form-control-sm" name="beacons.' + index + '.callsign" id="beacons.' + index + '.callsign" value="' + escapeHtml(beacon.callsign) + '" oninput="this.value = this.value.toUpperCase();"><label for="beacons.' + index + '.callsign">Callsign</label></div>' +
            '<div class="form-floating field-profile"><input type="text" class="form-control form-control-sm" name="beacons.' + index + '.profileLabel" id="beacons.' + index + '.profileLabel" value="' + escapeHtml(beacon.profileLabel) + '"><label for="beacons.' + index + '.profileLabel">Profile Label</label></div>' +
            '<div class="form-floating field-comment"><input type="text" class="form-control form-control-sm" name="beacons.' + index + '.comment" id="beacons.' + index + '.comment" value="' + escapeHtml(beacon.comment) + '"><label for="beacons.' + index + '.comment">Comment</label></div>' +
            '<div class="form-floating field-status"><input type="text" class="form-control form-control-sm" name="beacons.' + index + '.status" id="beacons.' + index + '.status" value="' + escapeHtml(beacon.status) + '"><label for="beacons.' + index + '.status">Status</label></div>' +
            '<div class="form-floating field-symbol"><input type="text" class="form-control form-control-sm" name="beacons.' + index + '.symbol" id="beacons.' + index + '.symbol" value="' + escapeHtml(beacon.symbol) + '"><label for="beacons.' + index + '.symbol">Symbol</label></div>' +
            '<div class="form-floating field-overlay"><input type="text" class="form-control form-control-sm" name="beacons.' + index + '.overlay" id="beacons.' + index + '.overlay" value="' + escapeHtml(beacon.overlay) + '"><label for="beacons.' + index + '.overlay">Overlay</label></div>' +
            '<div class="form-floating field-mice"><input type="text" class="form-control form-control-sm" name="beacons.' + index + '.micE" id="beacons.' + index + '.micE" value="' + escapeHtml(beacon.micE) + '"><label for="beacons.' + index + '.micE">Mic-E</label></div>' +
            '<div class="form-check form-switch field-smart-active"><input class="form-check-input" type="checkbox" name="beacons.' + index + '.smartBeaconActive" id="beacons.' + index + '.smartBeaconActive" value="1" ' + (beacon.smartBeaconActive ? "checked" : "") + '><label class="form-check-label" for="beacons.' + index + '.smartBeaconActive">Smart Beacon Active</label></div>' +
            '<div class="form-check form-switch field-gps-eco"><input class="form-check-input" type="checkbox" name="beacons.' + index + '.gpsEcoMode" id="beacons.' + index + '.gpsEcoMode" value="1" ' + (beacon.gpsEcoMode ? "checked" : "") + '><label class="form-check-label" for="beacons.' + index + '.gpsEcoMode">GPS Eco Mode</label></div>' +
            '<div class="field-smart-setting"><label for="beacons.' + index + '.smartBeaconSetting" class="form-label"><small>Smart Beacon Setting</small></label><select name="beacons.' + index + '.smartBeaconSetting" id="beacons.' + index + '.smartBeaconSetting" class="form-control"><option value="0" ' + (beacon.smartBeaconSetting == 0 ? "selected" : "") + '>Human/Runner (Slow Speed)</option><option value="1" ' + (beacon.smartBeaconSetting == 1 ? "selected" : "") + '>Bicycle (Mid Speed)</option><option value="2" ' + (beacon.smartBeaconSetting == 2 ? "selected" : "") + '>Car/Motorcycle (Fast Speed)</option></select><button type="button" class="btn btn-sm btn-outline-secondary mt-1" onclick="resetSmartBeaconDefaults(' + index + ')">Reset to profile defaults</button></div>' +
            '<div class="smartbeacon-params field-smart-params p-2 bg-light rounded"><small class="text-muted">Custom SmartBeacon parameters</small><div class="row">' +
            renderSmartBeaconInput(index, "sbSlowRate", "Slow Rate (s)", beacon.sbSlowRate || 120) + renderSmartBeaconInput(index, "sbFastRate", "Fast Rate (s)", beacon.sbFastRate || 60) + renderSmartBeaconInput(index, "sbMinSpeed", "Min Speed (km/h)", beacon.sbMinSpeed || 5) + renderSmartBeaconInput(index, "sbMaxSpeed", "Max Speed (km/h)", beacon.sbMaxSpeed || 70) + renderSmartBeaconInput(index, "sbMinTurnAngle", "Min Turn Angle", beacon.sbMinTurnAngle || 28) + renderSmartBeaconInput(index, "sbTurnSlope", "Turn Slope", beacon.sbTurnSlope || 240) + renderSmartBeaconInput(index, "sbMinBeaconTime", "Min Beacon Time (s)", beacon.sbMinBeaconTime || 60) + '</div></div></div>';
        container.appendChild(element);
    });
}

function renderLoraProfiles() {
    const container = byId("lora-settings");
    const freqMinMHz = (loraFrequencyLimits.min / 1000000).toFixed(0);
    const freqMaxMHz = (loraFrequencyLimits.max / 1000000).toFixed(0);
    container.innerHTML = '<div class="lora-actions d-flex justify-content-end mb-3"><button type="button" class="btn btn-outline-primary btn-sm" id="add-lora-profile">Add profile</button></div><input type="hidden" name="lora.count" id="lora.count" value="' + editableLoraProfiles.length + '">';
    editableLoraProfiles.forEach((lora, index) => {
        const element = document.createElement("div");
        const bw = numeric(lora.signalBandwidth, 125000);
        const sf = numeric(lora.spreadingFactor, 12);
        const cr = numeric(lora.codingRate4, 5);
        element.classList.add("row", "lora", "border-bottom", "py-2");
        element.innerHTML = '<div class="list-index mb-2"><strong>' + (index + 1) + ')</strong></div>' +
            '<div class="form-floating col-12 col-md-3 mb-2"><input type="text" class="form-control form-control-sm" name="lora.' + index + '.profileName" id="lora.' + index + '.profileName" value="' + escapeHtml(lora.profileName || ('PROFILE ' + (index + 1))) + '" maxlength="16" required><label for="lora.' + index + '.profileName">Profile name</label></div>' +
            '<div class="form-floating col-12 col-md-3 mb-2"><input type="number" class="form-control form-control-sm" name="lora.' + index + '.frequency" id="lora.' + index + '.frequency" value="' + numeric(lora.frequency, loraFrequencyLimits.min) + '" min="' + loraFrequencyLimits.min + '" max="' + loraFrequencyLimits.max + '" required><label for="lora.' + index + '.frequency">Freq (' + freqMinMHz + '-' + freqMaxMHz + ')</label></div>' +
            '<div class="form-floating col-6 col-md-2 mb-2"><select class="form-select form-select-sm" name="lora.' + index + '.signalBandwidth" id="lora.' + index + '.signalBandwidth"><option value="62500" ' + (bw === 62500 ? "selected" : "") + '>62.5</option><option value="125000" ' + (bw === 125000 ? "selected" : "") + '>125</option></select><label for="lora.' + index + '.signalBandwidth">BW (kHz)</label></div>' +
            '<div class="form-floating col-6 col-md-1 mb-2"><select class="form-select form-select-sm" name="lora.' + index + '.spreadingFactor" id="lora.' + index + '.spreadingFactor">' + [5,6,7,8,9,10,11,12].map((v) => '<option value="' + v + '" ' + (sf === v ? "selected" : "") + '>SF' + v + '</option>').join("") + '</select><label for="lora.' + index + '.spreadingFactor">SF</label></div>' +
            '<div class="form-floating col-6 col-md-1 mb-2"><select class="form-select form-select-sm" name="lora.' + index + '.codingRate4" id="lora.' + index + '.codingRate4">' + [5,6,7,8].map((v) => '<option value="' + v + '" ' + (cr === v ? "selected" : "") + '>4:' + v + '</option>').join("") + '</select><label for="lora.' + index + '.codingRate4">CR</label></div>' +
            '<div class="form-floating col-6 col-md-1 mb-2"><input type="number" class="form-control form-control-sm" name="lora.' + index + '.power" id="lora.' + index + '.power" value="' + numeric(lora.power, 20) + '" min="1" max="22" required><label for="lora.' + index + '.power">Power</label></div>' +
            '<div class="col-12 col-md-auto mb-2 d-flex align-items-center"><button type="button" class="btn btn-outline-danger" title="Delete profile" onclick="removeLoraProfile(' + index + ')">Delete</button></div>';
        container.appendChild(element);
    });
}
function removeLoraProfile(index) { if (editableLoraProfiles.length <= 1) { alert("At least one LoRa profile is required."); return; } editableLoraProfiles.splice(index, 1); renderLoraProfiles(); }
function addLoraProfile() { const template = editableLoraProfiles[0] || defaultLoraProfiles()[0] || {}; editableLoraProfiles.push({ profileName: 'PROFILE ' + (editableLoraProfiles.length + 1), frequency: template.frequency || loraFrequencyLimits.min, signalBandwidth: template.signalBandwidth || 125000, spreadingFactor: template.spreadingFactor || 12, codingRate4: template.codingRate4 || 5, power: template.power || 20 }); renderLoraProfiles(); }

function renderWifiAPs() {
    const container = byId("wifi-ap-settings");
    container.innerHTML = '<div class="wifi-actions d-flex justify-content-end mb-3"><button type="button" class="btn btn-outline-primary btn-sm" id="add-wifi-ap">Add profile</button></div><input type="hidden" name="wifi.AP.count" id="wifi.AP.count" value="' + editableWifiAPs.length + '">';
    editableWifiAPs.forEach((ap, index) => {
        const element = document.createElement("div");
        element.classList.add("row", "wifi-ap", "border-bottom", "py-2");
        element.innerHTML = '<div class="list-index mb-2"><strong>' + (index + 1) + ')</strong></div>' +
            '<div class="form-floating col-12 col-md-5 mb-2"><input type="text" class="form-control form-control-sm" name="wifi.AP.' + index + '.ssid" id="wifi.AP.' + index + '.ssid" value="' + escapeHtml(ap.ssid || "") + '"><label for="wifi.AP.' + index + '.ssid">SSID</label></div>' +
            '<div class="form-floating col-12 col-md-5 mb-2"><input type="password" class="form-control form-control-sm" name="wifi.AP.' + index + '.password" id="wifi.AP.' + index + '.password" value="' + escapeHtml(ap.password || "") + '"><label for="wifi.AP.' + index + '.password">Passphrase</label></div>' +
            '<div class="col-12 col-md-auto mb-2 d-flex align-items-center"><button type="button" class="btn btn-outline-danger" title="Delete profile" onclick="removeWifiAP(' + index + ')">Delete</button></div>';
        container.appendChild(element);
    });
}
function removeWifiAP(index) { if (editableWifiAPs.length <= 1) { alert("At least one WiFi profile is required."); return; } editableWifiAPs.splice(index, 1); renderWifiAPs(); }
function addWifiAP() { editableWifiAPs.push({ ssid: "", password: "" }); renderWifiAPs(); }

function loadSettings(settings) {
    currentSettings = settings;
    renderBeaconProfiles(settings.beacons || []);
    const other = settings.other || {};
    setChecked("simplifiedTrackerMode", other.simplifiedTrackerMode); setValue("sendCommentAfterXBeacons", other.sendCommentAfterXBeacons ?? 10); setValue("path", other.path ?? "WIDE1-1"); setValue("nonSmartBeaconRate", other.nonSmartBeaconRate ?? 15); setValue("rememberStationTime", other.rememberStationTime ?? 30); setValue("standingUpdateTime", other.standingUpdateTime ?? 15); setChecked("sendAltitude", other.sendAltitude); setChecked("disableGPS", other.disableGPS); setValue("email", other.email ?? "");
    const display = settings.display || {}; setChecked("display.turn180", display.turn180); setChecked("display.showSymbol", display.showSymbol); setChecked("display.ecoMode", display.ecoMode); setValue("display.timeout", display.timeout ?? 4);
    const bluetooth = settings.bluetooth || {}; setChecked("bluetooth.active", bluetooth.active); setValue("bluetooth.deviceName", bluetooth.deviceName ?? "LoRaTracker"); setChecked("bluetooth.useBLE", bluetooth.useBLE); setChecked("bluetooth.useKISS", bluetooth.useKISS); if (bluetooth.hasBTClassic === false && byId("btClassicOption")) byId("btClassicOption").style.display = "none"; updateBluetoothFields();
    const aprsIs = settings.aprs_is || {}; setChecked("aprs_is.active", aprsIs.active); setValue("aprs_is.server", aprsIs.server ?? "euro.aprs2.net"); setValue("aprs_is.port", aprsIs.port ?? 14580); setValue("aprs_is.passcode", aprsIs.passcode ?? "-1"); updateAprsIsFields();
    const loraConfig = settings.loraConfig || {}; setChecked("loraConfig.repeaterMode", loraConfig.repeaterMode); setChecked("loraConfig.sendInfo", loraConfig.sendInfo ?? true); setValue("loraConfig.digipeatAlias", loraConfig.digipeatAlias ?? "WIDE1-1"); updateDigipeaterFields();
    loraFrequencyLimits.min = settings.loraFreqMin || 100000000; loraFrequencyLimits.max = settings.loraFreqMax || 1000000000; editableLoraProfiles = (settings.lora || []).filter((profile) => profile.frequency >= loraFrequencyLimits.min && profile.frequency <= loraFrequencyLimits.max); if (!editableLoraProfiles.length) editableLoraProfiles = defaultLoraProfiles(); renderLoraProfiles();
    const battery = settings.battery || {}; setChecked("battery.sendVoltage", battery.sendVoltage); setChecked("battery.voltageAsTelemetry", battery.voltageAsTelemetry); setChecked("battery.sendVoltageAlways", battery.sendVoltageAlways); setChecked("battery.monitorVoltage", battery.monitorVoltage); setValue("battery.sleepVoltage", numeric(battery.sleepVoltage, 2.9).toFixed(1)); updateBatteryFields();
    const telemetry = settings.telemetry || {}; setChecked("telemetry.active", telemetry.active); setChecked("telemetry.sendTelemetry", telemetry.sendTelemetry); setValue("telemetry.temperatureCorrection", numeric(telemetry.temperatureCorrection, 0).toFixed(1)); updateTelemetryFields();
    const winlink = settings.winlink || {}; setValue("winlink.password", winlink.password ?? "");
    const wifi = settings.wifi || {}; editableWifiAPs = (wifi.AP && wifi.AP.length ? wifi.AP : [{}]).map((ap) => ({ ssid: ap.ssid || "", password: ap.password || "" })); renderWifiAPs(); setValue("wifi.autoAP.password", wifi.autoAP?.password ?? "1234567890");
    const notification = settings.notification || {}; setChecked("notification.ledTx", notification.ledTx); setValue("notification.ledTxPin", notification.ledTxPin ?? 13); setChecked("notification.ledMessage", notification.ledMessage); setValue("notification.ledMessagePin", notification.ledMessagePin ?? 2); setChecked("notification.ledFlashlight", notification.ledFlashlight); setValue("notification.ledFlashlightPin", notification.ledFlashlightPin ?? 14); setChecked("notification.buzzerActive", notification.buzzerActive); setValue("notification.buzzerPinTone", notification.buzzerPinTone ?? 33); setValue("notification.buzzerPinVcc", notification.buzzerPinVcc ?? 25); setValue("notification.volume", notification.volume ?? 50); byId("volumeValue").textContent = notification.volume ?? 50; setChecked("notification.bootUpBeep", notification.bootUpBeep); setChecked("notification.txBeep", notification.txBeep); setChecked("notification.messageRxBeep", notification.messageRxBeep); setChecked("notification.stationBeep", notification.stationBeep); setChecked("notification.lowBatteryBeep", notification.lowBatteryBeep); setChecked("notification.shutDownBeep", notification.shutDownBeep); updateNotificationFields();
    const ptt = settings.pttTrigger || {}; setChecked("ptt.active", ptt.active); setChecked("ptt.reverse", ptt.reverse); setValue("ptt.preDelay", ptt.preDelay ?? 0); setValue("ptt.postDelay", ptt.postDelay ?? 0); setValue("ptt.io_pin", ptt.io_pin ?? 4); updatePttFields();
}
function updateBluetoothFields() { const active = byId("bluetooth.active").checked; setDisabled("bluetooth.deviceName", !active); setDisabled("bluetooth.useBLE", !active); setDisabled("bluetooth.useKISS", !active); }
function updateAprsIsFields() { const active = byId("aprs_is.active").checked; setDisabled("aprs_is.server", !active); setDisabled("aprs_is.port", !active); setDisabled("aprs_is.passcode", !active); }
function updateDigipeaterFields() { const active = byId("loraConfig.repeaterMode").checked; setDisabled("loraConfig.digipeatAlias", !active); }
function updateBatteryFields() { const sendVoltage = byId("battery.sendVoltage").checked; const monitorVoltage = byId("battery.monitorVoltage").checked; setDisabled("battery.voltageAsTelemetry", !sendVoltage); setDisabled("battery.sendVoltageAlways", !sendVoltage); setDisabled("battery.sleepVoltage", !monitorVoltage); }
function updateTelemetryFields() { const active = byId("telemetry.active").checked; setDisabled("telemetry.sendTelemetry", !active); setDisabled("telemetry.temperatureCorrection", !active); }
function updateNotificationFields() { const ledTx = byId("notification.ledTx").checked; const ledMessage = byId("notification.ledMessage").checked; const ledFlashlight = byId("notification.ledFlashlight").checked; const buzzer = byId("notification.buzzerActive").checked; setDisabled("notification.ledTxPin", !ledTx); setDisabled("notification.ledMessagePin", !ledMessage); setDisabled("notification.ledFlashlightPin", !ledFlashlight); setDisabled("notification.buzzerPinTone", !buzzer); setDisabled("notification.buzzerPinVcc", !buzzer); setDisabled("notification.volume", !buzzer); ["notification.bootUpBeep", "notification.txBeep", "notification.messageRxBeep", "notification.stationBeep", "notification.lowBatteryBeep", "notification.shutDownBeep"].forEach((id) => setDisabled(id, !buzzer)); }
function updatePttFields() { const active = byId("ptt.active").checked; setDisabled("ptt.reverse", !active); setDisabled("ptt.preDelay", !active); setDisabled("ptt.postDelay", !active); setDisabled("ptt.io_pin", !active); }
function resetSmartBeaconDefaults(index) { const defaults = [[120, 60, 5, 15, 28, 240, 12], [120, 60, 5, 40, 28, 240, 12], [120, 60, 10, 70, 28, 240, 10]]; const setting = parseInt(byId('beacons.' + index + '.smartBeaconSetting').value, 10) || 0; const selected = defaults[setting] || defaults[0]; ["sbSlowRate", "sbFastRate", "sbMinSpeed", "sbMaxSpeed", "sbMinTurnAngle", "sbTurnSlope", "sbMinBeaconTime"].forEach((field, fieldIndex) => { const element = byId('beacons.' + index + '.' + field); if (element) element.value = selected[fieldIndex]; }); }

document.getElementById("backup").onclick = backupSettings;
document.getElementById("restore").onclick = function (event) { event.preventDefault(); document.querySelector("input[type=file]").click(); };
document.querySelector("input[type=file]").onchange = function () { const files = document.querySelector("input[type=file]").files; if (!files.length) return; const reader = new FileReader(); reader.readAsText(files.item(0)); reader.onload = () => { try { loadSettings(JSON.parse(reader.result)); } catch (err) { console.error(err); alert("Invalid configuration backup"); } }; };
document.getElementById("reboot").addEventListener("click", function (event) { event.preventDefault(); fetch("/action?type=reboot", { method: "POST" }); showToast("Your device will be rebooted in a while"); });
document.addEventListener("click", function (event) { if (event.target && event.target.id === "add-lora-profile") addLoraProfile(); });
document.addEventListener("click", function (event) { if (event.target && event.target.id === "add-wifi-ap") addWifiAP(); });
document.getElementById("bluetooth.active").addEventListener("change", updateBluetoothFields);
document.getElementById("aprs_is.active").addEventListener("change", updateAprsIsFields);
document.getElementById("loraConfig.repeaterMode").addEventListener("change", updateDigipeaterFields);
document.getElementById("loraConfig.digipeatAlias").addEventListener("input", function () { this.value = this.value.toUpperCase(); });
document.getElementById("battery.sendVoltage").addEventListener("change", updateBatteryFields);
document.getElementById("battery.monitorVoltage").addEventListener("change", updateBatteryFields);
document.getElementById("telemetry.active").addEventListener("change", updateTelemetryFields);
document.getElementById("notification.ledTx").addEventListener("change", updateNotificationFields);
document.getElementById("notification.ledMessage").addEventListener("change", updateNotificationFields);
document.getElementById("notification.ledFlashlight").addEventListener("change", updateNotificationFields);
document.getElementById("notification.buzzerActive").addEventListener("change", updateNotificationFields);
document.getElementById("notification.volume").addEventListener("input", function () { byId("volumeValue").textContent = this.value; });
document.getElementById("ptt.active").addEventListener("change", updatePttFields);

const form = document.querySelector("form");
const saveModal = new bootstrap.Modal(document.getElementById("saveModal"), { backdrop: "static", keyboard: false });
const savedModal = new bootstrap.Modal(document.getElementById("savedModal"), {});
function checkConnection() { const controller = new AbortController(); setTimeout(() => controller.abort(), 2000); fetch("/status?_t=" + Date.now(), { signal: controller.signal }).then(() => { saveModal.hide(); savedModal.show(); setTimeout(function () { savedModal.hide(); }, 3000); fetchSettings(); }).catch(() => { setTimeout(checkConnection, 0); }); }
form.addEventListener("submit", async (event) => { event.preventDefault(); fetch(form.action, { method: form.method, body: new FormData(form) }); saveModal.show(); setTimeout(checkConnection, 2000); });
fetchSettings();
applyBuildInfoPreviewFallback();
