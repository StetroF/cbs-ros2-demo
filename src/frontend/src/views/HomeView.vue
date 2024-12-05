<script setup>
</script>

<template>
  <div>
    <header>
      <div class="header-icons">
        <div class="icon company-icon">
          <!-- <img src="path/to/company-icon.png" alt="Company Icon" /> -->
        </div>

        <router-link to="/">
          <button class="icon home-button">Home Page</button>
        </router-link>

        <router-link to="/Nav">
          <button class="icon setting-button">Books</button>
        </router-link>

        <router-link to="/user">
          <button class="icon setting-button">用户</button>
        </router-link>
      </div>
    </header>

    <aside class="sidebar">
      <h2>侧边栏</h2>
      <ul>
        <li><a href="#">菜单项3</a></li>
        <button type="button" class="btn btn-warning btn-sm" @click="onNavClick()">导航栏</button>
      </ul>
    </aside>
    <main>
          <router-view></router-view> 
    </main>
  </div>
</template>

