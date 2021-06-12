function blobToPngBase64(blob) {
  const imageBlob = new Blob([blob], {type: 'image/png'});
  return URL.createObjectURL(imageBlob);
}

function startStreaming() {
  const pngStreamElement = document.querySelector("#png-stream")
  const socket = new WebSocket(`ws://${location.hostname}:9002`);
  socket.onmessage = async function (event) {
    pngStreamElement.src = blobToPngBase64(event.data);
  }
}

window.addEventListener('load', (_) => startStreaming());
