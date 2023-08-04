R"(
<!DOCTYPE html>
<html>
<body>
  <div class="container">
    <div class="box">
      <h3>Update Firmware</h3>
      <form method="POST" action="#" enctype="multipart/form-data" id="upload_form">
        <input type="file" name="update">
        <input type="submit" value="Update">
      </form>
      <div id="prg">progress: 0%</div>
    </div>
  </div>
  <script>
    function uploadFirmware() {
      var form = document.getElementById("upload_form");
      var fileInput = document.querySelector('input[type="file"]');
      var progressBar = document.getElementById("prg");

      var xhr = new XMLHttpRequest();
      xhr.open("POST", "/update");

      xhr.upload.addEventListener("progress", function (evt) {
        if (evt.lengthComputable) {
          var progress = (evt.loaded / evt.total) * 100;
          progressBar.innerHTML = "progress: " + Math.round(progress) + "%";
        }
      }, false);

      xhr.onreadystatechange = function () {
        if (xhr.readyState === 4) {
          if (xhr.status === 200) {
            alert("Firmware upload successful!");
          } else {
            alert("Firmware upload failed.");
          }
        }
      };

      var formData = new FormData(form);
      formData.append("update", fileInput.files[0]);

      xhr.send(formData);
    }

    document.addEventListener("DOMContentLoaded", function () {
      var form = document.getElementById("upload_form");
      form.addEventListener("submit", function (e) {
        e.preventDefault();
        uploadFirmware();
      });
    });
  </script>
</body>
</html>
)"