
const electron        = require('electron');
const fs              = require('fs'); 
window.d3             = require('d3');
const functionPlot    = require('function-plot');





document.addEventListener('DOMContentLoaded', function() {
  let options = {};
  let elems = document.querySelectorAll('.dropdown-trigger');
  let instances = M.Dropdown.init(elems, options);
});




document.addEventListener('DOMContentLoaded', function() {
  var elems = document.querySelectorAll('.dropdown-trigger');
  var instances = M.Dropdown.init(elems, options);
});




function handle_deadbands(){
  let db_ub = document.getElementById('db_ub');
  let db_lb = document.getElementById('db_lb');

  if(db_ub.value >= 1520){ db_ub.value = 1520; }
  if(db_ub.value <= 1502){ db_ub.value = 1502; }

  if(db_lb.value >= 1498){ db_lb.value = 1498; }
  if(db_lb.value <= 1480){ db_lb.value = 1480; }

  rate_rendering();
}



function handle_toggles(caller){
  let min_rpm_toggle    = document.getElementById('min_spin_rpm');
  let acro_toggle       = document.getElementById('acro_toggle');
  let stabilized_toggle = document.getElementById('stabilized_toggle');

  if(min_rpm_toggle.checked){
    document.getElementById('min_rpm').disabled = false;
  }

  if(min_rpm_toggle.checked == false){
    handle_min_rpm();
  }

  if(caller == "acro_toggle"){
    if(acro_toggle.checked){
      stabilized_toggle.checked = false;
      document.getElementById('gyro_a').disabled   = true;
      document.getElementById('accel_a').disabled  = true;
      document.getElementById('dr2').disabled     = true;
      document.getElementById('sel_dr2').innerHTML = "Select";
    }else{
      stabilized_toggle.checked = true;
      document.getElementById('gyro_a').disabled  = false;
      document.getElementById('accel_a').disabled = false;
      document.getElementById('dr2').disabled    = false;
    }
  }

  if(caller == "stabilized_toggle"){
    if(stabilized_toggle.checked){
      acro_toggle.checked = false;
      document.getElementById('gyro_a').disabled  = false;
      document.getElementById('accel_a').disabled = false;
      document.getElementById('dr2').disabled    = false;
    }else{
      acro_toggle.checked = true;
      document.getElementById('gyro_a').disabled   = true;
      document.getElementById('accel_a').disabled  = true;
      document.getElementById('dr2').disabled     = true;
      document.getElementById('sel_dr2').innerHTML = "Select";
    }    
  }

}



function handle_min_rpm(){
  let min_rpm_toggle = document.getElementById('min_spin_rpm');

  if(min_rpm_toggle.checked == false){ 
    document.getElementById('min_rpm').value = 1010;
    document.getElementById('min_rpm').disabled = true;
   }
}



function handle_compl_flt_alpha(caller){
  let gyro_alpha  = document.getElementById('gyro_a'); 
  let accel_alpha = document.getElementById('accel_a');

  if(caller == "gyro_a"){
    if(gyro_alpha.value >= 1){ gyro_alpha.value = 1; }
    if(gyro_alpha.value <= 0){ gyro_alpha.value = 0; }
    accel_alpha.value = (1 - gyro_alpha.value).toFixed(3);
  }

  if(caller == "accel_a"){
    if(accel_alpha.value >= 1){ accel_alpha.value = 1; }
    if(accel_alpha.value <= 0){ accel_alpha.value = 0; }
    gyro_alpha.value = (1 - accel_alpha.value).toFixed(3);
  }
}  




function change_controller_speed(caller){
  if(caller == "250"){ document.getElementById('sel_dr1').innerHTML =  "250 Hz"; }
  if(caller == "500"){ document.getElementById('sel_dr1').innerHTML =  "500 Hz"; }
  if(caller ==   "1"){ document.getElementById('sel_dr1').innerHTML =   "1 KHz"; }
  if(caller ==   "2"){ document.getElementById('sel_dr1').innerHTML =   "2 KHz"; }
  if(caller ==   "4"){ document.getElementById('sel_dr1').innerHTML =   "4 KHz"; }
  if(caller ==   "8"){ document.getElementById('sel_dr1').innerHTML =   "8 KHz"; }
}




function change_bounding_angle(caller){
  if(caller == "10"){ document.getElementById('sel_dr2').innerHTML =  "10 &deg"; }
  if(caller == "15"){ document.getElementById('sel_dr2').innerHTML =  "15 &deg"; }
  if(caller == "35"){ document.getElementById('sel_dr2').innerHTML =  "35 &deg"; }
  if(caller == "45"){ document.getElementById('sel_dr2').innerHTML =  "45 &deg"; }
}




function rate_rendering(){
  let pitch_rate   = document.getElementById('pitch_rate').value;
  let roll_rate    = document.getElementById('roll_rate').value;
  let yaw_rate     = document.getElementById('yaw_rate').value;
  let bandwidth_ub = document.getElementById('db_ub').value - 1500;
  let bandwidth_lb = -(1500 - document.getElementById('db_lb').value);
  let max_y        = 800;

  if(pitch_rate <= 0.2){ document.getElementById('pitch_rate').value = 0.2; pitch_rate = 0.2; }
  if(roll_rate  <= 0.2){ document.getElementById('roll_rate').value  = 0.2; roll_rate  = 0.2; }
  if(yaw_rate   <= 0.2){ document.getElementById('yaw_rate').value   = 0.2; yaw_rate   = 0.2; }

  if(bandwidth_lb == -bandwidth_ub){
    document.getElementById('p_rate').innerHTML = "Rate: "+Math.round((500-bandwidth_ub)/pitch_rate)+" &deg/s";
    document.getElementById('r_rate').innerHTML = "Rate: "+Math.round((500-bandwidth_ub)/roll_rate)+" &deg/s";
    document.getElementById('y_rate').innerHTML = "Rate: "+Math.round((500-bandwidth_ub)/yaw_rate)+" &deg/s";
  }else{
    let avg_rate = ( Math.round((500-bandwidth_ub)/pitch_rate) + Math.round((500+bandwidth_lb)/pitch_rate) ) / 2; 
    document.getElementById('p_rate').innerHTML = "Rate: "+avg_rate+" &deg/s";

    avg_rate = ( Math.round((500-bandwidth_ub)/roll_rate) + Math.round((500+bandwidth_lb)/roll_rate) ) / 2; 
    document.getElementById('p_rate').innerHTML = "Rate: "+avg_rate+" &deg/s";

    avg_rate = ( Math.round((500-bandwidth_ub)/yaw_rate) + Math.round((500+bandwidth_lb)/yaw_rate) ) / 2; 
    document.getElementById('p_rate').innerHTML = "Rate: "+avg_rate+" &deg/s";
  }

  functionPlot({
    target: '#function_area',
    width: 520,
    height: 320,
    disableZoom: true,
    xAxis: { label: 'Raw Radio', domain: [-500, 500] },
    yAxis: { label: 'Rotation Rate', domain: [-max_y, max_y] },

    data: [
      { fn: 'x /'+pitch_rate, range: [-500,         bandwidth_lb], color: 'green', graphType: 'polyline' },
      { fn: '0'             , range: [bandwidth_lb, bandwidth_ub], color: 'green', graphType: 'polyline' },
      { fn: 'x /'+pitch_rate, range: [bandwidth_ub,          500], color: 'green', graphType: 'polyline' },

      { fn: 'x /'+roll_rate, range: [-500,          bandwidth_lb], color: 'blue',  graphType: 'polyline' },
      { fn: '0'            , range: [bandwidth_lb,  bandwidth_ub], color: 'blue',  graphType: 'polyline' },
      { fn: 'x /'+roll_rate, range: [bandwidth_ub,           500], color: 'blue',  graphType: 'polyline' },

      { fn: 'x /'+yaw_rate, range: [-500,           bandwidth_lb], color: 'red',   graphType: 'polyline' },
      { fn: '0'           , range: [bandwidth_lb,   bandwidth_ub], color: 'red',   graphType: 'polyline' },
      { fn: 'x /'+yaw_rate, range: [bandwidth_ub,            500], color: 'red',   graphType: 'polyline' },
    ]
  });

}




function initialize(){
  load_configuration();
  handle_toggles('acro_toggle');
  handle_toggles('stabilized_toggle');
  handle_toggles('min_spin_rpm');
  rate_rendering();

  

}



function save_data_to_file(){
  let min_rpm_toggle    = document.getElementById('min_spin_rpm');
  let acro_toggle       = document.getElementById('acro_toggle');
  let stabilized_toggle = document.getElementById('stabilized_toggle');

  let logger = fs.createWriteStream('STM32F303K8T6 Drone/Inc/config.h');

  logger.write('#ifndef CONFIG_H_\n');
  logger.write('#define CONFIG_H_\n\n');

  logger.write('\t#include "common.h"\n\n');

  logger.write('\t#define KP_PITCH ' + document.getElementById('pitch_p').value + '\n');
  logger.write('\t#define KI_PITCH ' + document.getElementById('pitch_i').value + '\n');
  logger.write('\t#define KD_PITCH ' + document.getElementById('pitch_d').value + '\n\n');

  logger.write('\t#define KP_ROLL ' + document.getElementById('roll_p').value + '\n');
  logger.write('\t#define KI_ROLL ' + document.getElementById('roll_i').value + '\n');
  logger.write('\t#define KD_ROLL ' + document.getElementById('roll_d').value + '\n\n');

  logger.write('\t#define KP_YAW ' + document.getElementById('yaw_p').value + '\n');
  logger.write('\t#define KI_YAW ' + document.getElementById('yaw_i').value + '\n');
  logger.write('\t#define KD_YAW ' + document.getElementById('yaw_d').value + '\n\n');

  logger.write('\t#define DEADBAND_UPPER_BOUND ' + document.getElementById('db_ub').value + '\n');
  logger.write('\t#define DEADBAND_LOWER_BOUND ' + document.getElementById('db_lb').value + '\n\n');

  if(min_rpm_toggle.checked){
    logger.write('\t#define THROTTLE_LOWER_THD ' + document.getElementById('min_rpm').value + '\n\n');
  }

  logger.write('\t#define ALPHA_VALUE ' + document.getElementById('gyro_a').value + '\n\n');

  if(acro_toggle.checked){
    logger.write('\t#define BOUNDING_ANGLE 0\n');
  }

  if(stabilized_toggle.checked){
    let angle = document.getElementById('sel_dr2').innerHTML.split(" ")[0];
    
    if(angle == "10"){ logger.write('\t#define BOUNDING_ANGLE ANGLE_CORRECTION_10\n'); }
    if(angle == "15"){ logger.write('\t#define BOUNDING_ANGLE ANGLE_CORRECTION_15\n'); }
    if(angle == "35"){ logger.write('\t#define BOUNDING_ANGLE ANGLE_CORRECTION_35\n'); }
    if(angle == "45"){ logger.write('\t#define BOUNDING_ANGLE ANGLE_CORRECTION_45\n'); }
  }

  let control_speed = document.getElementById('sel_dr1').innerHTML.split(" ")[0];
  
  if(control_speed == "250"){ logger.write('\t#define CONTROLLER_SPEED LOOP_250HZ\n'); }
  if(control_speed == "500"){ logger.write('\t#define CONTROLLER_SPEED LOOP_490HZ\n'); }
  if(control_speed ==   "1"){ logger.write('\t#define CONTROLLER_SPEED LOOP_1KHZ\n');  }
  if(control_speed ==   "2"){ logger.write('\t#define CONTROLLER_SPEED LOOP_2KHZ\n');  }
  if(control_speed ==   "4"){ logger.write('\t#define CONTROLLER_SPEED LOOP_4KHZ\n');  }
  if(control_speed ==   "8"){ logger.write('\t#define CONTROLLER_SPEED LOOP_8KHZ\n');  }


  logger.write('\t#define SETPOINT_PITCH_SPEED_RATE ' + parseFloat(document.getElementById('pitch_rate').value).toFixed(1) + '\n');
  logger.write('\t#define SETPOINT_ROLL_SPEED_RATE '  + parseFloat(document.getElementById('roll_rate').value).toFixed(1)  + '\n');
  logger.write('\t#define SETPOINT_YAW_SPEED_RATE '   + parseFloat(document.getElementById('yaw_rate').value).toFixed(1) + '\n\n');

  logger.write('\n\n#endif');

  logger.end();
}



function dump_configuration(){
  let data = {
                      "pitch_p"                   : document.getElementById('pitch_p').value,
                      "pitch_i"                   : document.getElementById('pitch_i').value,
                      "pitch_d"                   : document.getElementById('pitch_d').value,
                      "roll_p"                    : document.getElementById('roll_p').value,
                      "roll_i"                    : document.getElementById('roll_i').value,
                      "roll_d"                    : document.getElementById('roll_d').value,
                      "yaw_p"                     : document.getElementById('yaw_p').value,
                      "yaw_i"                     : document.getElementById('yaw_i').value,
                      "yaw_d"                     : document.getElementById('yaw_d').value,
                      "db_ub"                     : document.getElementById('db_ub').value,
                      "db_lb"                     : document.getElementById('db_lb').value,
                      "stabilized_toggle"         : document.getElementById('stabilized_toggle').checked,
                      "acro_toggle"               : document.getElementById('acro_toggle').checked,
                      "min_spin_rpm"              : document.getElementById('min_spin_rpm').checked,
                      "min_rpm"                   : document.getElementById('min_rpm').value,
                      "gyro_a"                    : document.getElementById('gyro_a').value,
                      "sel_dr2"                   : document.getElementById('sel_dr2').innerHTML,
                      "sel_dr1"                   : document.getElementById('sel_dr1').innerHTML,
                      "pitch_rate"                : parseFloat(document.getElementById('pitch_rate').value).toFixed(1),
                      "roll_rate"                 : parseFloat(document.getElementById('roll_rate').value).toFixed(1),
                      "yaw_rate"                  : parseFloat(document.getElementById('yaw_rate').value).toFixed(1)        
                    };

  data = JSON.stringify(data);
  fs.writeFileSync('dump/config.json', data);                
}



function load_configuration(){
  let data = fs.readFileSync('dump/config.json');
  data = JSON.parse(data);

  for(var key in data){
    if(key == "stabilized_toggle" || key == "acro_toggle" || key == "min_spin_rpm"){
      document.getElementById(key).checked = data[key]; 
    }else if(document.getElementById(key).tagName == "INPUT"){ 
      console.log(data[key]);
      document.getElementById(key).value = data[key]; 
    }else{
      document.getElementById(key).innerHTML = data[key]; 
    }
  }

}




function flash_firmware(){
  save_data_to_file();
  dump_configuration();
}