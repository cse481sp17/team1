$(function() {
  $(document).ready(function() {

      // Helpers
      var ERROR = "Unexpected error: please reload the page and try again"
      var handleError = function() {
        // Woops no good
        console.log("Errreur");
        $('.response', buttonGroup).addClass('error');
        $('.response', buttonGroup).html("Errreur");
        $('.response', buttonGroup).show();
        $('.loading', buttonGroup).hide();
      };
      var createOption = function(type, item) {
        return "<li><input id='" + type + "-" + item + "' type='radio' name='" + type + "' value='" + item + "'><label for='" + type + "-" + item + "'>" + item + "</label></li>";
      };
      var sleep = function(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
      }
      var getFoodAndLocations = function() {
        // Fetch fresh hot food options (straight fire)
        $.ajax({
          url: 'http://localhost:8080',
          type: 'GET',
          dataType: 'json',
          crossDomain: true,
          cache: false,
          timeout: 5000,
          success: function(data) {
            GETCallback(data);
          },
          error: function() {
            handleError();
            sleep(5000).then(getFoodAndLocations);
          }
        })};

      // Page elements is right here
      var form = $('#selectionForm');
      var foodGroup = $('#foodGroup', form);
      var foodOptions = $('#food', foodGroup);
      var sideGroup = $('#sideGroup', form);
      var sideOptions = $('#sides', sideGroup);
      var locationGroup = $('#locationGroup', form);
      var locationOptions = $('#locations', locationGroup);
      var buttonGroup = $('.buttonGroup', form);

      // Init
      $('.response', buttonGroup).hide();
      $('.loading', buttonGroup).hide();
      getFoodAndLocations();

      // Callbacks
      var GETCallback = function(data) {
        $('.response', buttonGroup).hide();

        // Put hot, steamy food options on the page
        data['food'].forEach(function(item) {
          console.log("Food item:", item);
          foodOptions.append(createOption('foodItem', item));
        });

        // Add sides to page
        data['sides'].forEach(function(item) {
          console.log("Side item:", item);
          sideOptions.append(createOption('sideItem', item));
        });

        // Make locations happen
        data['locations'].forEach(function(loc) {
          console.log("Location:", loc);
          locationOptions.append(createOption('location', loc));
        });

        // Hide the dank spinner gifs
        $('.loading', foodGroup).hide();
        $('.loading', sideGroup).hide();
        $('.loading', locationGroup).hide();
      };

      var POSTCallback = function(response) {
        if (response === 'success') {
          console.log("Food has delivered");
          $('.response', buttonGroup).removeClass('error');
          $('.response', buttonGroup).html("Food is been delivering");
          $('.response', buttonGroup).show();
          $('.loading', buttonGroup).hide();
        } else {
          handleError();
          return;
        }
      };

      form.submit(function(e) {
        e.preventDefault();
        var selectedItem = $('input[name=foodItem]:checked', foodOptions).val();
        var selectedSide = $('input[name=sideItem]:checked', sideOptions).val();
        var selectedLoc = $('input[name=location]:checked', locationOptions).val();
        console.log("Selected food item:", selectedItem);
        console.log("Selected side item:", selectedSide);
        console.log("Selected location:", selectedLoc);

        $('.loading', buttonGroup).show();

        if (!selectedItem || !selectedLoc) {
          console.log("Form is incomplete");
          handleError();
          return;
        }

        var foodAndLocation = {
          foodItem: selectedItem,
          sideItem: selectedSide,
          location: selectedLoc
        };

        console.log("Sending:", foodAndLocation);

        $.ajax({
          url: 'http://localhost:8080',
          type: 'POST',
          dataType: 'text',
          crossDomain: true,
          data: foodAndLocation,
          cache: false,
          success: function(response) {
            POSTCallback(response);
          },
          error: function() {
            handleError();
          }
        });
      });
  });
});
