"use strict";

const POLL_MS = 1200;
const COMMAND_REFRESH_MS = 180;
const ADMIN_PASSWORD = "ecovision"; // doar protectie vizuala, nu securitate reala

const $ = (id) => document.getElementById(id);
const toast = $("toast");
let currentMode = "unknown";
let pollBusy = false;

function showToast(message) {
  if (!toast) return;
  toast.textContent = message;
  toast.style.display = "block";
  clearTimeout(window.ecoToastTimer);
  window.ecoToastTimer = setTimeout(() => { toast.style.display = "none"; }, 2600);
}

function showSection(id) {
  document.querySelectorAll(".page-section").forEach((section) => {
    const active = section.id === id;
    section.classList.toggle("active", active);
    section.style.display = active ? "block" : "none";
  });
  document.querySelectorAll(".nav-btn").forEach((button) => {
    button.classList.toggle("active", button.dataset.section === id);
  });
  document.body.classList.remove("mobile-menu-open");
  const overlay = $("mobileMenuOverlay");
  if (overlay) overlay.setAttribute("aria-hidden", "true");
  window.scrollTo({ top: 0, behavior: "smooth" });
}

document.querySelectorAll("[data-section]").forEach((button) => {
  button.addEventListener("click", () => showSection(button.dataset.section));
});

const menuBtn = $("menuBtn");
const mobileOverlay = $("mobileMenuOverlay");
if (menuBtn && mobileOverlay) {
  menuBtn.addEventListener("click", (event) => {
    event.preventDefault();
    event.stopPropagation();
    const open = document.body.classList.toggle("mobile-menu-open");
    mobileOverlay.setAttribute("aria-hidden", String(!open));
    menuBtn.textContent = open ? "×" : "☰";
  });
  document.addEventListener("click", (event) => {
    if (!mobileOverlay.contains(event.target) && event.target !== menuBtn) {
      document.body.classList.remove("mobile-menu-open");
      mobileOverlay.setAttribute("aria-hidden", "true");
      menuBtn.textContent = "☰";
    }
  });
}

function normalizeBaseUrl(value) {
  const clean = String(value || "").trim().replace(/\/+$/, "");
  if (!clean) return "";
  return /^https?:\/\//i.test(clean) ? clean : `http://${clean}`;
}

function robotBaseUrl() {
  return normalizeBaseUrl(localStorage.getItem("ecovisionRobotIp"));
}

function loadSettings() {
  const saved = localStorage.getItem("ecovisionRobotIp") || "";
  if ($("robotIpInput")) $("robotIpInput").value = saved;
  if ($("robotEndpointInput")) $("robotEndpointInput").value = "/control?cmd=";
  if ($("savedIpText")) {
    $("savedIpText").textContent = saved ? `Adresă salvată: ${saved}` : "Adresă nesetată.";
  }

  const cameraIp = localStorage.getItem("ecovisionCameraIp") || "";
  if ($("cameraIpInput")) $("cameraIpInput").value = cameraIp;
  updateCamera(cameraIp);
}

if ($("saveIpBtn")) {
  $("saveIpBtn").addEventListener("click", () => {
    const entered = $("robotIpInput") ? $("robotIpInput").value.trim() : "";
    localStorage.setItem("ecovisionRobotIp", entered.replace(/^https?:\/\//i, "").replace(/\/+$/, ""));
    loadSettings();
    fetchRobotData();
    showToast("Adresa robotului a fost salvată.");
  });
}

function updateCamera(value) {
  const stream = $("cameraStream");
  const placeholder = $("cameraPlaceholder");
  if (!stream || !placeholder) return;
  const base = normalizeBaseUrl(value);
  if (base) {
    stream.src = `${base}/stream`;
    stream.style.display = "block";
    placeholder.style.display = "none";
  } else {
    stream.removeAttribute("src");
    stream.style.display = "none";
    placeholder.style.display = "block";
  }
}

if ($("saveCameraBtn")) {
  $("saveCameraBtn").addEventListener("click", () => {
    const value = $("cameraIpInput") ? $("cameraIpInput").value.trim() : "";
    localStorage.setItem("ecovisionCameraIp", value.replace(/^https?:\/\//i, "").replace(/\/+$/, ""));
    loadSettings();
    showToast("Adresa camerei a fost salvată.");
  });
}

function setConnection(online, message) {
  if ($("connectionState")) $("connectionState").textContent = online ? "ONLINE" : "OFFLINE";
  if ($("lastUpdate")) $("lastUpdate").textContent = message;
}

function setText(id, text) {
  const element = $(id);
  if (element) element.textContent = text;
}

function airQualityFromMq135(reading) {
  const value = Number(reading);
  if (!Number.isFinite(value)) return { label: "Necunoscută", status: "Aștept o măsurătoare validă." };
  if (value < 350) return { label: "Bună", status: "Calitatea aerului este bună." };
  if (value < 700) return { label: "Medie", status: "Calitatea aerului este moderată." };
  return { label: "Scăzută", status: "Calitatea aerului necesită atenție." };
}

function updateMode(mode) {
  currentMode = String(mode || "unknown").toLowerCase();
  document.querySelectorAll(".mode-btn").forEach((button) => {
    button.classList.toggle("active", button.dataset.mode === currentMode);
  });
  setText("modeStatus", `Mod confirmat: ${currentMode === "auto" ? "autonom" : currentMode === "manual" ? "manual" : "necunoscut"}`);
  document.querySelectorAll(".control-btn[data-command]:not([data-command='stop'])").forEach((button) => {
    button.disabled = currentMode !== "manual";
  });
}

function updateSensorView(data) {
  if (data.temp !== undefined) setText("tempValue", `${Number(data.temp).toFixed(1)} °C`);
  if (data.hum !== undefined) setText("humValue", `${Number(data.hum).toFixed(1)} %`);
  if (data.mq7 !== undefined) setText("coValue", `${data.mq7}`);
  if (data.mq8 !== undefined) setText("h2Value", `${data.mq8}`);
  if (data.mq135 !== undefined) setText("airValue", `${data.mq135}`);
  if (data.dust !== undefined) setText("dustValue", `${Number(data.dust).toFixed(1)} µg/m³`);
  if (data.pressure !== undefined) setText("pressureValue", `${Number(data.pressure).toFixed(1)} hPa`);

  const air = airQualityFromMq135(data.mq135);
  setText("airScore", air.label);
  setText("airStatus", air.status);
  if (data.mode !== undefined) updateMode(data.mode);
}

async function fetchRobotData() {
  if (pollBusy) return;
  const base = robotBaseUrl();
  if (!base) {
    setConnection(false, "Setează adresa ESP32 în Admin.");
    return;
  }
  pollBusy = true;
  try {
    const response = await fetch(`${base}/data`, { cache: "no-store", signal: AbortSignal.timeout(1000) });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    if (data.online === false) {
      setConnection(false, "ESP32 este online, dar Mega nu trimite date.");
      return;
    }
    updateSensorView(data);
    setConnection(true, `Actualizat: ${new Date().toLocaleTimeString("ro-RO")}`);
  } catch (error) {
    setConnection(false, "Roverul nu răspunde.");
  } finally {
    pollBusy = false;
  }
}

async function sendRobotCommand(command, silent = false) {
  const base = robotBaseUrl();
  if (!base) {
    if (!silent) showToast("Setează adresa ESP32 în Admin.");
    return false;
  }
  try {
    const response = await fetch(`${base}/control?cmd=${encodeURIComponent(command)}`, {
      method: "POST",
      cache: "no-store",
      signal: AbortSignal.timeout(1000)
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    if (!silent) showToast(`Comandă trimisă: ${command}`);
    return true;
  } catch (error) {
    if (!silent) showToast("Comanda nu a ajuns la robot.");
    return false;
  }
}

document.querySelectorAll(".mode-btn").forEach((button) => {
  button.addEventListener("click", async () => {
    const mode = button.dataset.mode;
    const ok = await sendRobotCommand(mode);
    if (ok) {
      updateMode(mode);
      if (mode === "manual") await sendRobotCommand("stop", true);
    }
  });
});

function setLastCommand(text) {
  setText("lastCommand", text);
}

function bindManualControls() {
  document.querySelectorAll(".control-btn[data-command], .danger-btn[data-command]").forEach((button) => {
    const command = button.dataset.command;
    if (!command) return;
    if (command === "stop") {
      button.addEventListener("click", async (event) => {
        event.preventDefault();
        setLastCommand("Comandă: STOP");
        await sendRobotCommand("stop");
      });
      return;
    }

    let timer = null;
    const start = async (event) => {
      event.preventDefault();
      if (currentMode !== "manual" || timer) return;
      setLastCommand(`Comandă activă: ${command}`);
      await sendRobotCommand(command, true);
      timer = setInterval(() => sendRobotCommand(command, true), COMMAND_REFRESH_MS);
    };
    const stop = async (event) => {
      if (event) event.preventDefault();
      if (!timer) return;
      clearInterval(timer);
      timer = null;
      setLastCommand("Comandă: STOP");
      await sendRobotCommand("stop", true);
    };
    button.addEventListener("pointerdown", start);
    button.addEventListener("pointerup", stop);
    button.addEventListener("pointercancel", stop);
    button.addEventListener("pointerleave", stop);
  });
}

window.addEventListener("blur", () => {
  if (currentMode === "manual") sendRobotCommand("stop", true);
});
document.addEventListener("visibilitychange", () => {
  if (document.hidden && currentMode === "manual") sendRobotCommand("stop", true);
});

if ($("adminLoginBtn")) {
  const unlock = () => {
    if ($("adminPassword").value === ADMIN_PASSWORD) {
      $("adminLogin").classList.add("hidden");
      $("adminPanel").classList.remove("hidden");
      showToast("Panou admin deblocat.");
    } else {
      showToast("Parolă greșită.");
    }
  };
  $("adminLoginBtn").addEventListener("click", unlock);
  $("adminPassword").addEventListener("keydown", (event) => {
    if (event.key === "Enter") unlock();
  });
}

const CONTACT_EMAIL = "ecovisionapp53@gmail.com";
const CONTACT_SUBJECT = "Mesaj EcoVision";
const CONTACT_BODY = "Salut! Am o întrebare despre EcoVision.";

function gmailWebComposeUrl() {
  const params = new URLSearchParams({
    view: "cm",
    fs: "1",
    to: CONTACT_EMAIL,
    su: CONTACT_SUBJECT,
    body: CONTACT_BODY
  });
  return `https://mail.google.com/mail/?${params.toString()}`;
}

function gmailContactUrl() {
  const webFallback = gmailWebComposeUrl();
  if (!/Android/i.test(navigator.userAgent)) return webFallback;

  const subject = encodeURIComponent(CONTACT_SUBJECT);
  const body = encodeURIComponent(CONTACT_BODY);
  const fallback = encodeURIComponent(webFallback);
  return `intent://ecovisionapp53@gmail.com?subject=${subject}&body=${body}` +
         `#Intent;scheme=mailto;package=com.google.android.gm;` +
         `S.browser_fallback_url=${fallback};end`;
}

if ($("mailBtn")) {
  $("mailBtn").href = gmailContactUrl();
  if (/Android/i.test(navigator.userAgent)) $("mailBtn").removeAttribute("target");
}

if ($("copyEmailBtn")) {
  $("copyEmailBtn").addEventListener("click", async () => {
    try {
      await navigator.clipboard.writeText(CONTACT_EMAIL);
      showToast("Email copiat.");
    } catch (error) {
      showToast(CONTACT_EMAIL);
    }
  });
}

loadSettings();
bindManualControls();
updateMode("unknown");
showSection("home");
fetchRobotData();
setInterval(fetchRobotData, POLL_MS);
