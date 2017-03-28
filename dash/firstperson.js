
var imageNr = 0; // Serial number of current image
var webcam;

function startStream() {
  webcam = document.getElementById("webcam");

  setInterval(nextImage, 20);
}


function nextImage() {
  var img = new Image();
  img.style.position = "absolute";
  img.style.zIndex = -1;
  img.style.width = "100%";
  img.style.height = "100%";
  var imageZIndexClosure = imageNr;
  img.onload = function() {
    this.style.zIndex = imageZIndexClosure;
    while (this.previousElementSibling != null) {
      this.parentElement.removeChild(this.previousElementSibling);
    }
  };
  img.src = "http://roborio-254-frc.local:5801/?action=snapshot&n=" + (++imageNr);
  webcam.appendChild(img);

}
