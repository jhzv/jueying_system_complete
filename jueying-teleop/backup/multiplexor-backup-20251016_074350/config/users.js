// config/users.js
// Usuarios y credenciales para el sistema
// Passwords hasheados con bcrypt (rounds=10)

module.exports = {
  users: {
    admin: {
      username: 'admin',
      // password: 'admin123'
      passwordHash: '$2b$10$c46gZaxj4m8zj6PbMgcaWu1O2cEDWqF/.n2w.T/iHI8Zp1Z4kho5u',
      role: 'admin',
      permissions: ['control', 'view', 'configure']
    },
    operator1: {
      username: 'operator1',
      // password: 'password123'
      passwordHash: '$2b$10$wYUQEe8i8JxdCUs3QQhzQ.Hn4x1FeTDEY5rnJO6h5YjHeE2FYnjwu',
      role: 'operator',
      permissions: ['control', 'view']
    }
  }
};
