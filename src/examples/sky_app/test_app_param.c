/**
 * test_app_int //描述
 *
 * test_app_int //在qgc中显示的描述
 *
 * @min 0  //最小值
 * @max 100  //最大值
 * @unit m  //单位
 * @group Test  //该参数的分组
 */
 PARAM_DEFINE_INT32(TEST_APP_INT, 1);

 /**
  * test_app_float //描述
  *
  * test_app_float //在qgc中显示的描述
  *
  * @min 0  //最小值
  * @max 100  //最大值
  * @decimal 2  //小数精度为2
  * @increment 0.01  //增量步长为0.01
  * @unit m  //单位
  * @group Test  //该参数的分组
  */
 PARAM_DEFINE_FLOAT(TEST_APP_FLOAT, 0.1f);
