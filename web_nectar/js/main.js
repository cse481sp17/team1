$(function() {
  $(document).ready(function() {

      // Helpers
      var ERROR = "Unexpected error: please reload the page and try again"
      var handleError = function() {
        // Woops no good
        console.log("Please select your preference for ALL of the food option categories!");
        $('.response', buttonGroup).addClass('error');
        $('.response', buttonGroup).html("Please select your preference for each of the food option categories!");
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
      var dessertGroup = $('#dessertGroup', form);
      var dessertOptions = $('#dessert', dessertGroup);
      var sideGroup = $('#sideGroup', form);
      var sideOptions = $('#sides', sideGroup);
      var drinkGroup = $('#drinkGroup', form);
      var drinkOptions = $('#drinks', drinkGroup);
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

        // Add desserts to page
        data['dessert'].forEach(function(item) {
          console.log("Dessert item:", item);
          dessertOptions.append(createOption('dessertItem', item));
        });

        // Add drinks to page
        data['drinks'].forEach(function(item) {
          console.log("Dessert item:", item);
          drinkOptions.append(createOption('drinkItem', item));
        });

        // Make locations happen
        data['locations'].forEach(function(loc) {
          console.log("Location:", loc);
          locationOptions.append(createOption('location', loc));
        });

        // Hide the dank spinner gifs
        $('.loading', foodGroup).hide();
        $('.loading', sideGroup).hide();
        $('.loading', dessertGroup).hide();
        $('.loading', drinkGroup).hide();
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
        var selectedDessert = $('input[name=dessertItem]:checked', dessertOptions).val();
        var selectedDrink = $('input[name=drinkItem]:checked', drinkOptions).val();
        var selectedLoc = $('input[name=location]:checked', locationOptions).val();
        console.log("Selected food item:", selectedItem);
        console.log("Selected side item:", selectedSide);
        console.log("Selected dessert item:", selectedDessert);
        console.log("Selected drink item:", selectedDrink);
        console.log("Selected location:", selectedLoc);

        $('.loading', buttonGroup).show();

        if (!selectedItem || !selectedLoc || !selectedSide || !selectedDessert || !selectedDrink) {
          console.log("Form is incomplete");
          handleError();
          return;
        }

        var foodAndLocation = {
          foodItem: selectedItem,
          sideItem: selectedSide,
          dessertItem: selectedDessert,
          drinkItem: selectedDrink,
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
