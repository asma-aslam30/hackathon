/**
 * Custom NavbarItem Component Types
 * This file registers custom navbar item types for Docusaurus
 */
import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import TranslateNavbarItem from './TranslateNavbarItem';

export default {
  ...ComponentTypes,
  'custom-translate': TranslateNavbarItem,
};
