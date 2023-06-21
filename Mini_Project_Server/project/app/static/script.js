//get time
const monthNames = [
  "January", "February", "March",
  "April", "May", "June",
  "July", "August", "September",
  "October", "November", "December"
];

const dayNames = [
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
];

function getDaySuffix(day) {
  if (day >= 11 && day <= 13) {
    return day + "th";
  }
  switch (day % 10) {
    case 1:
      return day + "st";
    case 2:
      return day + "nd";
    case 3:
      return day + "rd";
    default:
      return day + "th";
  }
}

var date = new Date();
var dayName = dayNames[date.getDay()];
var day = getDaySuffix(date.getDate());
var month = monthNames[date.getMonth()];
var year = date.getFullYear();

window.addEventListener('load', function() {
  var gdate = document.getElementById("date");
  gdate.innerHTML = dayName + ", " + day + " " + month + " " + year;
})

//mqtt socket
var cnt = 0;
var datatemp = [];
var datahumi = [];
var chart;

var socket = io.connect("localhost:" + location.port);
socket.on('mqtt_message', function (data) {
  var temp = document.getElementById("temp-val");
  var humi = document.getElementById("humi-val");
  var show_value = document.querySelector('#show-value');
  
  var str = data.payload;
  var len = str.length;
  var temp_val = str.substring(0, parseInt(len/2));
  var humi_val = str.substring(parseInt(len/2), len);
  
  temp.innerHTML = parseInt(temp_val);
  humi.innerHTML = parseInt(humi_val);
  show_value.classList.add('open');

  if (cnt < 1000) {
    datatemp.push(parseInt(temp_val));
    datahumi.push(parseInt(humi_val));
    cnt = cnt + 1;
  } else {
    datatemp.shift();
    datahumi.shift();
    datatemp.push(parseInt(temp_val));
    datahumi.push(parseInt(humi_val));
  }

  chart.series[0].setData(datatemp);
  chart.series[1].setData(datahumi);
  
  if (cnt < 15) {
    var xAxis = chart.xAxis[0];
    var minValue = xAxis.dataMin;
    var maxValue = xAxis.dataMax;
    xAxis.setExtremes(minValue, maxValue);
  }
});

document.addEventListener("DOMContentLoaded", function () {
  chart = Highcharts.chart('chart', {
    chart: {
        type: 'spline'
    },
    title: {
        text: 'Chart'
    },
    plotOptions: {
      series: {
          lineWidth: 4,
          marker: {
              enabled: true,
              borderRadius: 10
          },
          states: {
              hover: {
                  lineWidth: 5
              }
          },
          linecap: 'round'
      }
    },
    xAxis: {
      // tickInterval: 1,
      type: 'linear',
      tickInterval: 1,
      min: 0,
      max: 15
    },
    scrollbar: {
      enabled: true
    },
    yAxis: {
      title: {
        text: 'Temperature - Humidity'
      }
    },
    series: [ {
        name: 'Temperature',
        data: datatemp
        // data: []
      },
      {
        name: 'Humidity',
        color: '#00794b',
        data: datahumi
        // data: []
      }]
  })
});