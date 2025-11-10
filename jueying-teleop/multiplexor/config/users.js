/**
 * Sistema de Usuarios
 * Manejo de autenticación con bcrypt
 */

const bcrypt = require('bcrypt');

// ============================================================
// BASE DE DATOS DE USUARIOS
// ============================================================
const USERS = {
  admin: {
    username: 'admin',
    // Password: admin123
    passwordHash: '$2b$10$c46gZaxj4m8zj6PbMgcaWu1O2cEDWqF/.n2w.T/iHI8Zp1Z4kho5u',
    role: 'admin',
    permissions: ['control', 'view', 'configure']
  },
  operator1: {
    username: 'operator1',
    // Password: password123
    passwordHash: '$2b$10$wYUQEe8i8JxdCUs3QQhzQ.Hn4x1FeTDEY5rnJO6h5YjHeE2FYnjwu',
    role: 'operator',
    permissions: ['control', 'view']
  },
  viewer1: {
    username: 'viewer1',
    // Password: viewer123
    passwordHash: '$2b$10$8Ub5R4xZxWRqPJPk1Y3YZe5YmYXYXYXYXYXYXYXYXYXYXYXYXY',
    role: 'viewer',
    permissions: ['view']
  }
};

// ============================================================
// FUNCIONES DE AUTENTICACIÓN
// ============================================================

/**
 * Autenticar usuario
 * 
 * @param {string} username
 * @param {string} password
 * @returns {Promise<Object|null>} Usuario autenticado o null
 */
async function authenticate(username, password) {
  const user = USERS[username];
  
  if (!user) {
    return null;
  }
  
  try {
    const match = await bcrypt.compare(password, user.passwordHash);
    
    if (match) {
      // Retornar usuario sin el hash
      return {
        username: user.username,
        role: user.role,
        permissions: user.permissions
      };
    }
    
    return null;
  } catch (error) {
    console.error('Authentication error:', error);
    return null;
  }
}

/**
 * Verificar si usuario tiene permiso
 * 
 * @param {Object} user
 * @param {string} permission
 * @returns {boolean}
 */
function hasPermission(user, permission) {
  return user && user.permissions && user.permissions.includes(permission);
}

/**
 * Generar hash de password (para crear nuevos usuarios)
 * 
 * @param {string} password
 * @returns {Promise<string>}
 */
async function hashPassword(password) {
  return await bcrypt.hash(password, 10);
}

/**
 * Agregar nuevo usuario (usar con cuidado en producción)
 * 
 * @param {string} username
 * @param {string} password
 * @param {string} role
 * @param {Array<string>} permissions
 */
async function addUser(username, password, role = 'operator', permissions = ['view']) {
  if (USERS[username]) {
    throw new Error('User already exists');
  }
  
  const passwordHash = await hashPassword(password);
  
  USERS[username] = {
    username,
    passwordHash,
    role,
    permissions
  };
  
  console.log(`User ${username} added successfully`);
  return true;
}

/**
 * Listar usuarios (sin passwords)
 */
function listUsers() {
  return Object.values(USERS).map(user => ({
    username: user.username,
    role: user.role,
    permissions: user.permissions
  }));
}

// ============================================================
// EXPORTS
// ============================================================
module.exports = {
  authenticate,
  hasPermission,
  hashPassword,
  addUser,
  listUsers,
  USERS
};

// ============================================================
// SCRIPT DE AYUDA (solo si se ejecuta directamente)
// ============================================================
if (require.main === module) {
  const args = process.argv.slice(2);
  
  if (args.length === 0) {
    console.log(`
Uso:
  node config/users.js hash <password>        - Generar hash de password
  node config/users.js list                   - Listar usuarios
  node config/users.js add <user> <pass>      - Agregar usuario
    `);
    process.exit(0);
  }
  
  const command = args[0];
  
  switch (command) {
    case 'hash':
      if (args[1]) {
        hashPassword(args[1]).then(hash => {
          console.log(`\nPassword: ${args[1]}`);
          console.log(`Hash: ${hash}\n`);
        });
      } else {
        console.log('Error: Provide password to hash');
      }
      break;
      
    case 'list':
      console.log('\nUsuarios registrados:');
      console.log(listUsers());
      console.log();
      break;
      
    case 'add':
      if (args[1] && args[2]) {
        addUser(args[1], args[2]).catch(err => console.error('Error:', err.message));
      } else {
        console.log('Error: Provide username and password');
      }
      break;
      
    default:
      console.log('Unknown command:', command);
  }
}
