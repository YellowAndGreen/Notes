# Nuxt3

## 配置

### 环境分别配置

```ts
export default defineNuxtConfig({
  $production: {
    routeRules: {
      '/**': { isr: true }
    }
  },
  $development: {
    //
  }
})
```

### **runtimeConfig**

The `runtimeConfig` API exposes values like environment variables to the rest of your application. By default, these keys are only available server-side. The keys within `runtimeConfig.public` are also available client-side.

Those values should be defined in `nuxt.config` and can be overridden using environment variables.

```ts
export default defineNuxtConfig({
  runtimeConfig: {
    // The private keys which are only available server-side
    apiSecret: '123',
    // Keys within public are also exposed client-side
    public: {
      apiBase: '/api'
    }
  }
})
```

### [App Configuration](https://nuxt.com/docs/getting-started/configuration#app-configuration)

The `app.config.ts` file, located in the source directory (by default the root of the project), is used to expose public variables that can be determined at build time. Contrary to the `runtimeConfig` option, these can not be overridden using environment variables.

A minimal configuration file exports the `defineAppConfig` function containing an object with your configuration. The `defineAppConfig` helper is globally available without import.

app.config.ts



```ts
export default defineAppConfig({
  title: 'Hello Nuxt',
  theme: {
    dark: true,
    colors: {
      primary: '#ff0000'
    }
  }
})
```

These variables are exposed to the rest of your application using the [`useAppConfig`](https://nuxt.com/docs/api/composables/use-app-config) composable.

```vue
<script setup lang="ts">
const appConfig = useAppConfig()
</script>
```

## Views

### 组件

所有组件定义在components文件夹下，可以组件会在文件中自动导入，可直接使用。

### 页面

页面定义在pages文件夹下，每个位于 `pages/` 目录中的文件都代表着不同的路由，展示其内容。

要使用页面，创建一个 `pages/index.vue` 文件，并在 `app.vue` 中添加 `<NuxtPage />` 组件（或者删除 `app.vue` 以使用默认入口）。现在，你可以通过在 `pages/` 目录中添加新文件来创建更多页面及其对应的路由。

### [Layouts](https://nuxt.com/docs/getting-started/views#layouts)

**布局（Layouts）** 是页面的包装器，用于为多个页面提供共同的用户界面，例如页眉和页脚的显示。布局是使用 `<slot />` 组件来显示页面内容的 **Vue** 文件。默认情况下，`layouts/default.vue` 文件将被使用。你还可以在页面元数据中设置自定义布局。

如果你的应用程序中只有一个布局，我们建议使用 `app.vue` 并使用 `<NuxtPage />`。

```vue
<template>
  <div>
    <AppHeader />
    <slot />
    <AppFooter />
  </div>
</template>
```

## Assets

Nuxt uses two directories to handle assets like stylesheets, fonts or images.

- The [`public/`](https://nuxt.com/docs/guide/directory-structure/public) directory content is served at the server root as-is.
- The [`assets/`](https://nuxt.com/docs/guide/directory-structure/assets) directory contains by convention every asset that you want the build tool (Vite or webpack) to process.

在 **Nuxt 3** 中，`public/` 目录和 `assets/` 目录都用于处理资源文件，但它们之间有一些区别：

1. **public/ 目录**：
   - `public/` 目录中的文件会被原样复制到最终的构建目录中。
   - 这些文件不会经过构建工具（如 Vite 或 webpack）的处理。
   - 适合放置一些不需要处理的静态资源，例如第三方库、favicon.ico 等。
2. **assets/ 目录**：
   - `assets/` 目录中的文件会被构建工具处理，例如压缩、转换等。
   - 这些文件会被打包到最终的构建目录中。
   - 适合放置需要处理的资源，例如样式表、字体、图像等。

**[Public Directory](https://nuxt.com/docs/getting-started/assets#public-directory)**

The [`public/`](https://nuxt.com/docs/guide/directory-structure/public) directory is used as a public server for static assets publicly available at a defined URL of your application.

You can get a file in the [`public/`](https://nuxt.com/docs/guide/directory-structure/public) directory from your application's code or from a browser by the root URL `/`.

For example, referencing an image file in the `public/img/` directory, available at the static URL `/img/nuxt.png`:

```vue
<template>
  <img src="/img/nuxt.png" alt="Discover Nuxt 3" />
</template>
```

**[Assets Directory](https://nuxt.com/docs/getting-started/assets#assets-directory)**

Nuxt uses [Vite](https://vitejs.dev/guide/assets.html) (default) or [webpack](https://webpack.js.org/guides/asset-management) to build and bundle your application. The main function of these build tools is to process JavaScript files, but they can be extended through [plugins](https://vitejs.dev/plugins) (for Vite) or [loaders](https://webpack.js.org/loaders) (for webpack) to process other kind of assets, like stylesheets, fonts or SVG. This step transforms the original file mainly for performance or caching purposes (such as stylesheets minification or browser cache invalidation).

By convention, Nuxt uses the [`assets/`](https://nuxt.com/docs/guide/directory-structure/assets) directory to store these files but there is no auto-scan functionality for this directory, and you can use any other name for it.

In your application's code, you can reference a file located in the [`assets/`](https://nuxt.com/docs/guide/directory-structure/assets) directory by using the `~/assets/` path.

For example, referencing an image file that will be processed if a build tool is configured to handle this file extension:

```vue
<template>
  <img src="~/assets/img/nuxt.png" alt="Discover Nuxt 3" />
</template>
```





## Routing
Nuxt file-system routing creates a route for every file in the pages/ directory.

One core feature of Nuxt is the file system router. Every Vue file inside the pages/ directory creates a corresponding URL (or route) that displays the contents of the file. By using dynamic imports for each page, Nuxt leverages code-splitting to ship the minimum amount of JavaScript for the requested route.

### Pages
Nuxt routing is based on vue-router and generates the routes from every component created in the pages/ directory, based on their filename.

This file system routing uses naming conventions to create dynamic and nested routes:

Generated Router File

```
| pages/
---| about.vue
---| index.vue
---| posts/
-----| [id].vue
```

### Navigation
The `<NuxtLink>` component links pages between them. It renders an `<a>` tag with the href attribute set to the route of the page. Once the application is hydrated, page transitions are performed in JavaScript by updating the browser URL. This prevents full-page refreshes and allows for animated transitions.

When a `<NuxtLink>` enters the viewport on the client side, Nuxt will automatically prefetch components and payload (generated pages) of the linked pages ahead of time, resulting in faster navigation.

```js
pages/app.vue

<template>
  <header>
    <nav>
      <ul>
        <li><NuxtLink to="/about">About</NuxtLink></li>
        <li><NuxtLink to="/posts/1">Post 1</NuxtLink></li>
        <li><NuxtLink to="/posts/2">Post 2</NuxtLink></li>
      </ul>
    </nav>
  </header>
</template>
```

Read more in Docs > API > Components > Nuxt Link.

### Route Parameters

The useRoute() composable can be used in a `<script setup>` block or a setup() method of a Vue component to access the current route details.

```js
pages/posts/[id].vue

<script setup lang="ts">
const route = useRoute()

// When accessing /posts/1, route.params.id will be 1
console.log(route.params.id)
</script>
```

Read more in Docs > API > Composables > Use Route.

### Route Middleware

Nuxt provides a customizable route middleware framework you can use throughout your application, ideal for extracting code that you want to run before navigating to a particular route.

Route middleware runs within the Vue part of your Nuxt app. Despite the similar name, they are completely different from server middleware, which are run in the Nitro server part of your app.

There are three kinds of route middleware:

- Anonymous (or inline) route middleware, which are defined directly in the pages where they are used.
- Named route middleware, which are placed in the middleware/ directory and will be automatically loaded via asynchronous import when used on a page. (Note: The route middleware name is normalized to kebab-case, so someMiddleware becomes some-middleware.)
- Global route middleware, which are placed in the middleware/ directory (with a .global suffix) and will be automatically run on every route change.

Example of an auth middleware protecting the /dashboard page:

```js
pages/dashboard.vue

<script setup lang="ts">
definePageMeta({
  middleware: 'auth'
})
</script>

<template>
  <h1>Welcome to your dashboard</h1>
</template>
```

```ts
middleware/auth.ts

export default defineNuxtRouteMiddleware((to, from) => {
  // isAuthenticated() is an example method verifying if a user is authenticated
  if (isAuthenticated() === false) {
    return navigateTo('/login')
  }
})
```

### 全局中间件

在Nuxt3中，你可以通过以下两种方式对所有页面使用一个中间件：

+ 全局中间件：你可以通过在中间件文件名后添加.global后缀来创建全局中间件。例如，如果你的中间件文件名为auth.ts，则可以将其更改为auth.global.ts。这样，Nuxt就会在每次路由更改时自动运行此中间件。

+ 在nuxt.config.js中添加中间件：你也可以在nuxt.config.js的router配置中添加中间件，如下所示：

```
router: {
  middleware: ['your-global-middleware'],
},
```

这将查找你的middleware目录中的中间件，并将其应用于你的整个应用程序。

+ 作为插件加入

  ```js
  export default defineNuxtPlugin(() => {
    addRouteMiddleware('global-test', () => {
      console.log('this global middleware was added in a plugin and will be run on every route change')
    }, { global: true })
  
    addRouteMiddleware('named-test', () => {
      console.log('this named middleware was added in a plugin and would override any existing middleware of the same name')
    })
  })
  
  ```

  

### Route Validation

Nuxt offers route validation via the validate property in definePageMeta() in each page you wish to validate.

The validate property accepts the route as an argument. You can return a boolean value to determine whether or not this is a valid route to be rendered with this page. If you return false, and another match can’t be found, this will cause a 404 error. You can also directly return an object with statusCode/statusMessage to respond immediately with an error (other matches will not be checked).

If you have a more complex use case, then you can use anonymous route middleware instead.

pages/posts/[id].vue

```vue
<script setup lang="ts">
definePageMeta({
  validate: async (route) => {
    // Check if the id is made up of digits
    return /^\d+$/.test(route.params.id)
  }
})
</script>
```