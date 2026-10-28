export const $ = (selector, root = document) => root.querySelector(selector);
export const $$ = (selector, root = document) => [...root.querySelectorAll(selector)];

export const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

export function fmt(value, digits = 2) {
  if (value === null || value === undefined || Number.isNaN(Number(value))) return "--";
  return Number(value).toFixed(digits);
}

export function pad(value, length = 2) {
  return String(value).padStart(length, "0");
}

export function timeLabel(timestamp) {
  const date = new Date((timestamp || Date.now() / 1000) * 1000);
  return `${pad(date.getHours())}:${pad(date.getMinutes())}:${pad(date.getSeconds())}`;
}

export async function api(path, options = {}) {
  const response = await fetch(path, {
    method: options.method || "GET",
    credentials: "same-origin",
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    body: options.body === undefined ? undefined : JSON.stringify(options.body),
  });
  const text = await response.text();
  let payload = null;
  try {
    payload = text ? JSON.parse(text) : null;
  } catch {
    payload = { detail: text };
  }
  if (!response.ok) {
    const detail = payload && payload.detail ? payload.detail : response.statusText;
    const error = new Error(typeof detail === "string" ? detail : JSON.stringify(detail));
    error.status = response.status;
    throw error;
  }
  return payload;
}

let toastTimer = null;
export function toast(message, type = "") {
  const element = $("#toast");
  if (!element) return;
  element.textContent = message;
  element.className = `toast show ${type}`.trim();
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => {
    element.className = "toast";
  }, 3200);
}

export function setText(selector, text) {
  const element = $(selector);
  if (element) element.textContent = text;
}
