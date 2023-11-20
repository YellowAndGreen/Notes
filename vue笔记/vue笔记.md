# vue.js

## 基础知识

Vue.config：全局配置

可通过`Vue.config.productionTip = false`阻止 vue 在启动时生成生产提示。

favicon.ico：网站图标，放在根目录下

### 创建Vue容器

使用el绑定dom元素，data为传入的数据，在dom使用{{name}}来获取数据

```vue
//创建Vue实例
new Vue({
el:'#demo', //el用于指定当前Vue实例为哪个容器服务，值通常为css选择器字符串。
data:{ //data中用于存储数据，数据供el所指定的容器去使用，值我们暂时先写成一个对象。
name:'atguigu',
address:'北京'
}
})  
```

或使用v.mount('#root')来绑定元素

```
		/* const v = new Vue({
			//el:'#root', //第一种写法
			data:{
				name:'尚硅谷'
			}
		})
		console.log(v)
		v.$mount('#root') //第二种写法 */
```

### 模版语法

#### 插值语法{{data}}

#### 指令语法

1. 绑定标签属性v-bind（缩写为:）

   `<span v-bind:title="message">   11  </span>`

   <mark>v-bind是单向绑定，数据只能从data中传入，而不能从dom中传出</mark>

2. 绑定表单类元素v-model

   `<input type="text" v-model:value="name">`

   缩写为：

   `<input type="text" v-model="name"><br/>`

   <mark>v-model是双向绑定，dom和data中的数据同步</mark>

3. 使用v-on:xxx 或 @xxx 绑定事件（缩写为@）

​              1.事件的回调需要配置在methods对象中，最终会在vm上；

​              2.methods中配置的函数，不要用箭头函数！否则this就不是vm了；

​              3.methods中配置的函数，都是被Vue所管理的函数，this的指向是vm 或 组件实例对象；

​              4.@click="demo" 和 @click="demo($event)" 效果一致，但后者可以传参；

```html
		<div id="root">
			<h2>欢迎来到{{name}}学习</h2>
			<!-- <button v-on:click="showInfo">点我提示信息</button> -->
			<button @click="showInfo1">点我提示信息1（不传参）</button>
			<button @click="showInfo2($event,66)">点我提示信息2（传参）</button>
		</div>
```

```vue
		const vm = new Vue({
			el:'#root',
			data:{
				name:'尚硅谷',
			},
			methods:{
				showInfo1(event){
					// console.log(event.target.innerText)
					// console.log(this) //此处的this是vm
					alert('同学你好！')
				},
				showInfo2(event,number){
					console.log(event,number)
					// console.log(event.target.innerText)
					// console.log(this) //此处的this是vm
					alert('同学你好！！')
				}
			}
		})
```



### 属性

#### data

data属性有两种写法：

```
new Vue({
	el:'#root',
	//data的第一种写法：对象式
	/* data:{
		name:'尚硅谷'
	} */

	//data的第二种写法：函数式
	data(){
		console.log('@@@',this) //此处的this是Vue实例对象
		return{
			name:'尚硅谷'
		}
	}
})
```

### MVVM

1. M：模型(Model) ：对应data 中的数据
2. V：视图(View) ：模板
3. VM：视图模型(ViewModel) ： Vue 实例对象

因此一般用vm命名Vue 实例对象

![image-20211028211433269](C:\Users\Administrator\Desktop\WEB\笔记\vue\img\image-20211028211433269.png)

​			5.data中所有的属性，最后都出现在了vm身上。

​            6.vm身上所有的属性 及 Vue原型上所有属性，在Vue模板中都可以直接使用。

### 数据代理

<mark>数据代理：通过一个对象代理对另一个对象中属性的操作（读/写）</mark>

#### Object.defineProperty方法

```
let number = 18
let person = {
	name:'张三',
	sex:'男',
}

Object.defineProperty(person,'age',{
	// value:18,
	// enumerable:true, //控制属性是否可以枚举，默认值是false
	// writable:true, //控制属性是否可以被修改，默认值是false
	// configurable:true //控制属性是否可以被删除，默认值是false

	//当有人读取person的age属性时，get函数(getter)就会被调用，且返回值就是age的值
	get(){
		console.log('有人读取age属性了')
		return number
	},

	//当有人修改person的age属性时，set函数(setter)就会被调用，且会收到修改的具体值
	set(value){
		console.log('有人修改了age属性，且值是',value)
		number = value
	}

})
```



#### Vue中的数据代理

通过Object.defineProperty方法来实现，因此获取数据不需要{{_data.name}}而是{{name}}

只有data里的数据才会进行数据代理

![image-20211028214313265](C:\Users\Administrator\Desktop\WEB\笔记\vue\img\image-20211028214313265.png)

## 注意

1. js表达式 和 js代码(语句)的区别：

   1.表达式：<mark>**一个表达式会产生一个值**</mark>，可以放在任何一个需要值的地方：

   ​                  (1). a

   ​                  (2). a+b

   ​                  (3). demo(1)

   ​                  (4). x === y ? 'a' : 'b'

   

   2.js代码(语句)

   ​                  (1). if(){}

   ​                  (2). for(){}

## 其他
