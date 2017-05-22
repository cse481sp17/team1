$(function() {
  $(document).ready(function() {

    // Client page elements
    var clientContainer = $('#clientContainer');
    var selectionForm = $('form', clientContainer);
    var foodGroup = $('#foodGroup', selectionForm);
    var foodOptions = $('#food', foodGroup);
    var dessertGroup = $('#dessertGroup', selectionForm);
    var dessertOptions = $('#dessert', dessertGroup);
    var sideGroup = $('#sideGroup', selectionForm);
    var sideOptions = $('#sides', sideGroup);
    var drinkGroup = $('#drinkGroup', selectionForm);
    var drinkOptions = $('#drinks', drinkGroup);
    var locationGroup = $('#locationGroup', selectionForm);
    var locationOptions = $('#locations', locationGroup);
    var options = [foodOptions, dessertOptions, sideOptions, drinkOptions, locationOptions];

    // Chef page elements
    var chefContainer = $('#chefContainer');
    var chefForm = $('form', chefContainer);
    var orderQueue = $('#orderQueue', chefForm);

    // Helpers
    var fadeTimeout;
    var fadeOut = function(element) {
      if (fadeTimeout) {
        clearTimeout(fadeTimeout);
      }
      fadeTimeout = setTimeout(function() {element.fadeOut()}, 5000);
    }
    var sleep = function(ms) {
      return new Promise(resolve => setTimeout(resolve, ms));
    }

    var ERROR = "Unexpected error. Please reload the page and try again."
    var handleClientError = function(message) {
      if (!message) {
        message = ERROR;
      }
      console.log(message);
      $('.response', selectionForm).addClass('error');
      $('.response', selectionForm).html(message);
      $('.response', selectionForm).show();
      $('.loading', selectionForm).hide();
      fadeOut($('.response', selectionForm).show());
    };

    var handleChefError = function(message) {
      if (!message) {
        message = ERROR;
      }
      console.log(message);
      $('.response', chefForm).addClass('error');
      $('.response', chefForm).html(message);
      $('.response', chefForm).show();
      $('.loading', chefForm).hide();
      fadeOut($('.response', chefForm).show());
    }

    var createOption = function(type, item) {
      return $("<li><input id='" + type + "-" + item + "' type='radio' name='" + type + "' value='" + item + "'><label for='" + type + "-" + item + "'>" + item + "</label></li>");
    };

    var createChefOrder = function(order) {
      return $("<li><button id='order" + order.id + "' name='" + order.id + "'><strong>Order #" + order.id + ":</strong> " + order.foodItem + ", " + order.sideItem + ", " + order.dessertItem + ", " + order.drinkItem + "</li>");
    }

    var getData = function(type) {
      $.ajax({
        url: 'http://localhost:8080',
        type: 'GET',
        dataType: 'json',
        crossDomain: true,
        cache: false,
        timeout: 5000,
        success: function(data) {
          if (type === 'food') {
            populateFood(data);
          } else if (type === 'orders') {
            populateOrders(data);
          }
        },
        error: function() {
          sleep(5000).then(getData);
        }
      })
    };

    var deleteOrder = function(element) {
      var id = $('button', element).attr('name');
      $.ajax({
        url: 'http://localhost:8080?' + $.param({'id': id}),
        type: 'DELETE',
        crossDomain: true,
        cache: false,
        dataType: 'text',
        timeout: 5000,
        success: function(response) {
          console.log("here");
          orderFulfilledCallback(response, element, id);
        },
        error: function(e) {
          console.log(e);
          handleChefError();
        }
      })
    };

    // Callbacks
    var populateFood = function(data) {
      console.log("Received data:", data);

      // Put hot, steamy, main dish options on the page
      data['food'].forEach(function(item) {
        foodOptions.append(createOption('foodItem', item));
      });

      // Add sides to page
      data['sides'].forEach(function(item) {
        sideOptions.append(createOption('sideItem', item));
      });

      // Add desserts to page
      data['dessert'].forEach(function(item) {
        dessertOptions.append(createOption('dessertItem', item));
      });

      // Add drinks to page
      data['drinks'].forEach(function(item) {
        drinkOptions.append(createOption('drinkItem', item));
      });

      // Make locations happen
      data['locations'].forEach(function(loc) {
        locationOptions.append(createOption('location', loc));
      });

      // Hide the dank spinner gifs
      $('.loading', selectionForm).hide();
    };

    var populateOrders = function(data) {
      // Make orders happen
      data['orders'].forEach(function(order) {
        orderQueue.append(createChefOrder(order).click(function(e) {
          e.preventDefault();
          deleteOrder(this);
        }));
      });

      $('.loading', chefForm).hide();
    };

    var orderSubmitCallback = function(response) {
      if (response === 'success') {
        console.log("Food order placed");
        $('.response', selectionForm).removeClass('error');
        $('.response', selectionForm).html("Food order placed");
        $('.response', selectionForm).show();
        $('.buttonGroup .loading', selectionForm).hide();
        fadeOut($('.response', selectionForm));
      } else {
        handleClientError();
      }
    };

    var orderFulfilledCallback = function(response, element, id) {
      if (response === 'success') {
        message = "Food fulfillment received for order #" + id;
        console.log(message);
        element.remove();
        $('.response', chefForm).removeClass('error');
        $('.response', chefForm).html(message);
        $('.response', chefForm).show();
        $('.buttonGroup .loading', chefForm).hide();
        fadeOut($('.response', chefForm));
      } else {
        handleChefError();
      }
    };

    // Event handlers
    selectionForm.submit(function(e) {
      e.preventDefault();
      var selectedItem = $('input[name=foodItem]:checked', foodOptions).val();
      var selectedSide = $('input[name=sideItem]:checked', sideOptions).val();
      var selectedDessert = $('input[name=dessertItem]:checked', dessertOptions).val();
      var selectedDrink = $('input[name=drinkItem]:checked', drinkOptions).val();
      var selectedLoc = $('input[name=location]:checked', locationOptions).val();

      if (!selectedItem || !selectedLoc || !selectedSide || !selectedDessert || !selectedDrink) {
        handleClientError("Please select your preference for ALL of the food option categories!");
        return;
      }

      $('.buttonGroup .loading', selectionForm).show();
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
          orderSubmitCallback(response);
        },
        error: function() {
          handleClientError();
        }
      });
    });

    $('input.toggleInput#toggleClientPage').change(function() {
      displayClientPage();
    });

    $('input.toggleInput#toggleChefPage').change(function() {
      displayChefPage();
    });

    // Client page init
    var displayClientPage = function() {
      console.log("Switching to client page");

      // Show client page; hide chef page
      chefContainer.hide();
      clientContainer.show();

      // Clear entries
      options.forEach(function(ul) {
        ul.empty();
      });

      // Pull new data
      $('.response', selectionForm).hide();
      $('.loading', selectionForm).show();
      $('.buttonGroup .loading', selectionForm).hide();
      getData('food');
    }

    // Chef page init
    var displayChefPage = function() {
      console.log("Switching to chef page");

      // Show chef page; hide client page
      clientContainer.hide();
      chefContainer.show();

      // Clear entries
      orderQueue.empty();

      // Pull new data
      $('.response', chefForm).hide();
      $('.loading', chefForm).show();
      getData('orders');
    }

    // Init
    displayClientPage();
  });
});
